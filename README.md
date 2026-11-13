# Antares Micro R3 — Firmware di bordo

Firmware per RP2040 (scheda Waveshare RP2040 Zero) che acquisisce dati da
BMP280 (pressione/temperatura/altitudine) e MPU6050 (accelerometro +
giroscopio), registra un log **binario** ad alta frequenza su microSD e
trasmette dati leggibili via seriale. Gestisce un canale pirotecnico e una
striscia NeoPixel per lo stato visivo.

## Struttura del progetto

```
src/
├── main.cpp        # logica applicativa, task FreeRTOS, buffering SD
├── LogFormat.h     # formato binario del log (condiviso con tools/bin2csv.py)
├── BMP280.cpp/.h   # driver BMP280 su I2C software (SoftI2C)
└── SoftI2C.cpp/.h  # I2C bit-bang
tools/
├── bin2csv.py      # converte Logs/logN.bin in CSV (Python 3, no dipendenze)
└── test_bin2csv.py # test di round-trip del convertitore
platformio.ini      # ambiente PlatformIO (pico)
```

## Dipendenze (librerie)

- `Arduino.h` — core Arduino (RP2040 / arduino-pico)
- `FreeRTOS.h`, `task.h`, `semphr.h` — multitasking e mutex
- `Wire.h` — I2C hardware (MPU6050)
- `SPI.h` + `SD.h` — microSD su SPI1
- `NeoPixelBus.h` (makuna) — NeoPixel
- `cmath`, `cstring` — matematica (`powf`, `sqrtf`) e `memcpy`/`memset`
- `BMP280` / `SoftI2C` — moduli interni

## Pinout

| Segnale | GPIO | Note |
|---|---|---|
| BMP280 SDA | 15 | I2C software |
| BMP280 SCL | 14 | I2C software |
| MPU6050 SDA | 12 | I2C hardware (Wire) |
| MPU6050 SCL | 13 | I2C hardware (Wire) |
| SD SPI RX (MISO) | 8 | RP2040 SPI1: solo questa funzione |
| SD SPI TX (MOSI) | 11 | RP2040 SPI1: solo questa funzione |
| SD SPI SCK | 10 | |
| SD CS | 9 | |
| NeoPixel | 16 | |
| Pyro OUT | 28 | uscita canale pirotecnico |
| Pyro READ | 29 | ingresso analogico continuità |

## Architettura concorrente

| Task | Priorità | Stack | Periodo | Ruolo |
|---|---|---|---|---|
| `readSensors` | 3 | 4096 | ~10 ms | legge BMP280 + MPU6050, aggiorna lo stato condiviso |
| `react` | 2 | 8192 | 70 ms | macchina a stati di volo (Idle → Flight → Descent → Landed) |
| `writeLog` | 1 | 8192 | 10 ms | impacchetta i campioni e li scarica su SD a blocchi |
| `writeSerial` | 0 | 4096 | 60 ms | stampa misure leggibili su seriale |

Mutex:

- `xMutexSensors` — protegge le variabili di stato sensori.
- `xMutexSerial` — protegge l'accesso a `Serial`.

## Perché il log è binario e bufferizzato

Scrivere una riga CSV con `flush()` ad ogni campione costringeva la SD a un
sync per campione (~200 ms ciascuno), limitando il logging a ~5 Hz.

Adesso:

1. ogni campione è un record binario **fisso da 86 byte** (niente formattazione
   testuale, niente allocazioni sull'heap);
2. i record vengono accumulati in un buffer RAM da 4 KB (`LOG_BUFFER_SIZE`);
3. quando il buffer è pieno, o al massimo ogni 250 ms (`LOG_FLUSH_INTERVAL`),
   viene eseguita **una sola** `write()` contigua seguita da `flush()`.

A 100 Hz il buffer si riempie in ~450 ms: circa 2 trasferimenti al secondo
invece di decine, con perdita massima di `LOG_FLUSH_INTERVAL` in caso di
spegnimento improvviso.

## Formato del log binario

Definito in `src/LogFormat.h`, replicato in `tools/bin2csv.py`. Little endian.
Ogni record inizia con un frame header da 10 byte:

| Campo | Tipo | Valore |
|---|---|---|
| `magic` | u32 | `0x47534C41` ("ALSG") |
| `reserved` | u16 | 0 |
| `type` | u16 | `0x0048` ('H') header, `0x0053` ('S') campione |
| `size` | u16 | dimensione totale del record, frame header incluso |

### Record header ('H', 81 byte, uno per file)

| Campo | Tipo | Note |
|---|---|---|
| `version` | u8 | `LOG_VERSION` (1) |
| `reserved2` | u8 | 0 |
| `sample_interval_us` | u32 | periodo di campionamento previsto |
| `reserved3` | u8 | 0 |
| `build` | char[32] | stringa descrittiva |
| `note` | char[32] | stringa descrittiva |

### Record campione ('S', 86 byte)

| Campo | Tipo | Note |
|---|---|---|
| `timestamp_us` | u32 | `micros()` dall'avvio |
| `status` | u8 | 0 Idle, 1 Flight, 2 Descent, 3 Landed |
| `flags` | u8 | bit0 continuità, bit1 pyro (**campo autoritativo**) |
| `crc32` | u32 | 0 = non calcolato |
| `payload` | u8[64] | 15 float + 1 byte flags + 3 di padding (ordine qui sotto) |

#### Ordine dei dati dentro il payload

Il `payload` è un'immagine byte per byte di `snap.readings`, presa dallo
snapshot `SensorSnapshot` (`src/main.cpp`):

```cpp
// src/main.cpp
struct SensorSnapshot {
  float readings[READING_COUNT]; // READING_COUNT == LOG_FIELD_COUNT == 15
  float deltas[READING_COUNT];   // SOLO RAM: non viene mai scritto su SD
  uint8_t status;                // scritto nel campo status del record
};
```

`readings[]` è persistito su SD e nel CSV; `deltas[]` e `status` sono invece
disponibili solo a runtime (seriale e logica di volo).

L'ordine dei 15 float è fissato dagli indici `LOG_FIELD_*` di
`src/LogFormat.h` e da `takeSnapshot()`:

| Offset | Byte | Indice | Costante | Contenuto | Unità |
|---|---|---|---|---|---|
| 0 | 0 | 0 | `LOG_FIELD_ACCEL_X` | accelerazione X | g |
| 1 | 4 | 1 | `LOG_FIELD_ACCEL_Y` | accelerazione Y | g |
| 2 | 8 | 2 | `LOG_FIELD_ACCEL_Z` | accelerazione Z | g |
| 3 | 12 | 3 | `LOG_FIELD_ACCEL_TOT` | modulo accelerazione | g |
| 4 | 16 | 4 | `LOG_FIELD_GYRO_X` | velocità angolare X | °/s |
| 5 | 20 | 5 | `LOG_FIELD_GYRO_Y` | velocità angolare Y | °/s |
| 6 | 24 | 6 | `LOG_FIELD_GYRO_Z` | velocità angolare Z | °/s |
| 7 | 28 | 7 | `LOG_FIELD_ROT_X` | angolo accumulato X | ° |
| 8 | 32 | 8 | `LOG_FIELD_ROT_Y` | angolo accumulato Y | ° |
| 9 | 36 | 9 | `LOG_FIELD_ROT_Z` | angolo accumulato Z | ° |
| 10 | 40 | 10 | `LOG_FIELD_PRESSURE` | pressione | hPa |
| 11 | 44 | 11 | `LOG_FIELD_ALTITUDE` | altitudine assoluta | m |
| 12 | 48 | 12 | `LOG_FIELD_REL_ALTITUDE` | altitudine relativa | m |
| 13 | 52 | 13 | `LOG_FIELD_MAX_ALTITUDE` | altitudine massima | m |
| 14 | 56 | 14 | `LOG_FIELD_TEMPERATURE` | temperatura | °C |
| — | 60 | — | `LOG_FIELD_FLAGS_OFFSET` | copia informativa dei flag | bitmask |
| — | 61–63 | — | — | padding (zero) | — |

Note:

- I float sono **little endian** (Cortex-M0+), 4 byte ciascuno.
- Il campo `status` della struct **non** è dentro il payload: viene scritto
  nel campo `status` del record campione. L'ultimo byte del payload
  (offset 60) contiene una copia dei flag, tenuta per allineare i 15 float a
  4 byte; il campo `flags` del record resta quello autoritativo.
- L'ordine coincide con le colonne CSV prodotte da `tools/bin2csv.py`
  (dopo `Timestamp` e `Status`): se aggiungi un campo, aggiorna in blocco
  `LogFormat.h`, `takeSnapshot()`, `formatSerialLine()`, `bin2csv.py` e il
  `static_assert` in `main.cpp`.

#### Delta (solo RAM)

`snap.deltas[]` contiene, per **ogni** variabile di `readings[]` (stessi
indici `LOG_FIELD_*`), la variazione rispetto a `DELTA_WINDOW` cicli di
acquisizione fa (default 8 cicli, `DELTA_WINDOW` in `src/main.cpp`):

```cpp
float dAlt  = snap.deltas[LOG_FIELD_ALTITUDE];   // m cambiata in ~8 cicli
float dAcc  = snap.deltas[LOG_FIELD_ACCEL_TOT];  // g
```

- Calcolati da `updateDeltas()` dentro `readSensors`, sotto `xMutexSensors`,
  su una history circolare `deltaHistory[9][15]`.
- Il valore vengono copiato in `SensorSnapshot` da `takeSnapshot()`.
- Non vengono scritti nel record binario (nessun impatto sul formato del log);
  sono stampati in coda alla riga seriale dopo ` | d:` (AccelTOT, Altitude,
  Relative Altitude, Pressure, Temperature). Nessun costo su SD.

Il `magic` permette al convertitore di **riallinearsi** su file troncati o con
settori incompleti: i record non validi vengono saltati, non propagati.

## Conversion e lettura dei log

```bash
# un singolo file -> Logs/log0.csv
python3 tools/bin2csv.py Logs/log0.bin

# file e destinazione espliciti
python3 tools/bin2csv.py Logs/log0.bin volo1.csv

# tutta la cartella -> csv/
python3 tools/bin2csv.py Logs/ --out-dir csv/

# verifica del convertitore (round-trip su log sintetico)
python3 tools/test_bin2csv.py
```

Il CSV prodotto ha una riga di metadati `# build=... note=...` e 19 colonne:
`Timestamp, Status, AccelX, AccelY, AccelZ, AccelTOT, GyroX, GyroY, GyroZ,
RotX, RotY, RotZ, Pressure, Altitude, Relative Altitude, Max Altitude,
Temperature, Continuity, Pyro`.

## Variabili globali (`src/main.cpp`)

### Sincronizzazione
| Nome | Tipo | Mutex | Contenuto | Usata da |
|---|---|---|---|---|
| `xMutexSensors` | `SemaphoreHandle_t` | — | mutex dei sensori | tutti i task |
| `xMutexSerial` | `SemaphoreHandle_t` | — | mutex seriale | tutti i task |

### Sensori (stato condiviso, `xMutexSensors`)
| Nome | Tipo | Contenuto | Usata da |
|---|---|---|---|
| `temp` | `float` | temperatura BMP280 (°C) | `readSensors` (scrive), `takeSnapshot` |
| `pressure` | `float` | pressione (hPa) | `readSensors`, `takeSnapshot` |
| `altitude` | `float` | altitudine assoluta (m) | `readSensors`, `takeSnapshot` |
| `basealt` | `float` | altitudine di riferimento (m) | `readSensors`, `takeSnapshot` |
| `basealtSet` | `bool` | `basealt` già acquisita | `readSensors` |
| `maxalt` | `float` | altitudine massima raggiunta (m) | `readSensors`, `takeSnapshot` |
| `accel[3]` | `float` | accelerazione X,Y,Z (g) | `readSensors`, `takeSnapshot` |
| `accelTOT` | `float` | modulo dell'accelerazione (g) | `readSensors`, `takeSnapshot` |
| `gyro[3]` | `float` | velocità angolare (°/s) | `readSensors`, `takeSnapshot` |
| `rot[3]` | `float` | angoli accumulati (°) | `readSensors`, `takeSnapshot` |
| `status` | `uint8_t` | stato di volo: Idle(0), Flight(1), Descent(2), Landed(3) | `react` (scrive), `takeSnapshot` |
| `deltas[15]` | `float` | variazione di ogni lettura sugli ultimi `DELTA_WINDOW` cicli | `updateDeltas` (scrive), `takeSnapshot` |
| `deltaHistory[9][15]` | `float` | history circolare per il calcolo dei delta | `updateDeltas` (scrive), `deltaHead`, `deltaCount` |
| `deltaHead`, `deltaCount` | `size_t` | indice di scrittura e campioni validi nella history | `updateDeltas` |
| `readCycle_us` | `uint32_t` | durata ciclo lettura (µs) | `readSensors` (scrive), `writeSerial` |
| `maxRate_hz` | `float` | frequenza di lettura stimata | `readSensors` (scrive), `writeSerial` |
| `gyro_offset[3]` | `float` | offset giroscopio | `cgyro` (scrive), `readSensors` (legge) |

### Altro stato
| Nome | Tipo | Contenuto | Usata da |
|---|---|---|---|
| `softI2C` | `SoftI2C` | I2C software per il BMP280 | `bmp`, `setup` |
| `bmp` | `BMP280` | driver BMP280 (0x76) | `setup`, `readSensors` |
| `strip` | `NeoPixelBus` | striscia NeoPixel (1 LED) | `setup`, `cgyro`, `initMPU6050` |
| `logfile` | `String` | percorso del file `.bin` corrente | `setup`, `writeLog` |
| `beginTime` | `float` | istante di avvio (s) | `formatSerialLine` |

### Costanti principali
- Pinout e registri MPU6050 (`MPU6050_ADDR`, `REG_*`), scale (`ACCEL_SCALE`, `GYRO_SCALE`).
- Tempi: `READ_DELAY`, `WRITE_LOG_DELAY` (10 ms), `SERIAL_DELAY`, `REACT_DELAY`,
  `RECOVERY_INTERVAL`, `ERROR_MESSAGE_INTERVAL`.
- Buffer: `LOG_BUFFER_SIZE` (4096), `LOG_FLUSH_INTERVAL` (250 ms), `SERIAL_LINE_MAX`.
- Formato: da `LogFormat.h` (`LOG_MAGIC`, `LOG_FIELD_*`, `LOG_SAMPLE_RECORD_LEN`, …).

### Tipi
| Nome | Tipo | Contenuto |
|---|---|---|
| `SensorSnapshot` | `struct` | `float readings[LOG_FIELD_COUNT]` + `uint8_t status` |
| `log_file_header` | `struct` packed | record di intestazione (81 byte) |
| `log_sample_record` | `struct` packed | record campione (86 byte) |

## Funzioni

### Sensori e calibrazione
- `seaLevelPressure(alt, press)` — pressione al livello del mare (non usata nel flusso attuale).
- `cgyro(samples=2000)` — calibra il giroscopio, aggiorna `gyro_offset`.
- `initMPU6050()` — reset, wake-up, WHO_AM_I, configurazione ±16g/±2000°/s, `cgyro()`.
- `mpuWrite(reg, value)` — helper di scrittura registro MPU6050.
- `readWire16()` — legge due byte big-endian in `int16_t` (ordine di valutazione esplicito).
- `continuity()` — `true` se `PYRO_READ` > 100.

### Snapshot e formattazione
- `takeSnapshot(snap)` — copia lo stato condiviso in un `SensorSnapshot` (sotto mutex).
- `statusText(value)` — converte `LOG_STATUS_*` nella stringa leggibile.
- `formatSerialLine(snap, out, len)` — riga leggibile per la seriale (`snprintf`, nessun `String`).
- `createLogFile(path)` — crea `Logs/`, trova il primo `logN.bin` libero, scrive il record header.

### Task
- `readSensors` — acquisizione continua, aggiornamento di `maxalt`, recovery MPU6050 ogni 5 s.
- `writeLog` — snapshot → record binario → buffer RAM → scrittura SD a blocchi con flush periodico.
- `writeSerial` — stampa riga leggibile e timing.
- `react` — macchina a stati basata su valori **e** delta (le soglie sono in
  `src/main.cpp`). Ogni transizione è una combinazione di OR tra una condizione
  sul valore grezzo e una sui delta:

  | Transizione | Condizione |
  |---|---|
  | `Idle → Flight` | `accelTOT > LIFTOFF_ACCEL` **OR** `dAccelTOT > LIFTOFF_ACCEL_DELTA` |
  | `Flight → Descent` | `alt < maxalt − APOGEE_MARGIN_ALTITUDE` **OR** `dAlt < −APOGEE_ALTITUDE_DELTA` |
  | `Descent → Landed` | (`relAlt < LANDED_MARGIN_ALTITUDE` **OR** `|dAlt| < LANDED_ALTITUDE_DELTA`) **AND** `accelTOT < LANDED_ACCEL` |

  Dove `dX = snap.deltas[LOG_FIELD_X]` è la variazione sugli ultimi
  `DELTA_WINDOW` cicli (attualmente 8, ~5-10 campioni).
- `setStatus(newStatus)` — cambia lo stato globale sotto `xMutexSensors`.
- `updateDeltas()` — aggiorna history e deltas (solo `readSensors`, mutex tenuto).

### Setup
- `setup()` — mutex, seriale, NeoPixel, SoftI2C, microSD, file di log, BMP280,
  pyro, MPU6050, calibrazione, creazione dei task.
- `loop()` — vuoto.

## Note tecniche

- `src/LogFormat.h` e `tools/bin2csv.py` vanno modificati **insieme**: il
  convertitore replica byte per byte strutture e offset.
- I record non vengono mai spezzati tra due scritture; i file sono append-only
  e non richiedono una `seek` esplicita oltre a quella iniziale.
- Il CRC per record non è calcolato (risparmio di flash e tempo CPU): il
  convertitore lo verifica solo se diverso da zero.
- **Frequenza reale**: `writeLog` gira ogni 10 ms, ma `readSensors` impiega
  più di 10 ms per il ciclo BMP280+MPU6050 su I2C; per 100 Hz *distinti*
  serve parallelizzare l'acquisizione (I2C su core separato o riduzione dei
  tempi di bus). Il buffer SD non è il collo di bottiglia.
- `SD_SPI_RX`/`SD_SPI_TX` riflettono il ruolo SPI1 del RP2040: su GPIO 8/11
  l'unico mux valido è RX=8, TX=11.
- `build_flags = -DPIO_FRAMEWORK_ARDUINO_ENABLE_FREERTOS` in `platformio.ini`
  è necessario per esporre l'API FreeRTOS.
