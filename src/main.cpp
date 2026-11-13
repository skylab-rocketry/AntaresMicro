#include "BMP280.h"
#include "LogFormat.h"
#include "SoftI2C.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <NeoPixelBus.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <cmath>
#include <cstring>
#include <semphr.h>
#include <task.h>

// Pin mapping
constexpr uint8_t PYRO_OUT = 28;
constexpr uint8_t PYRO_READ = 29;
constexpr uint8_t BMP_SDA = 15;
constexpr uint8_t BMP_SCL = 14;
constexpr uint8_t MPU_SDA = 12;
constexpr uint8_t MPU_SCL = 13;
// SD card on SPI1. Named after the SPI1 role, not the SD signal: on RP2040
// these pins only support GPIO 8 = SPI1 RX (MISO) and GPIO 11 = SPI1 TX (MOSI).
constexpr uint8_t SD_SPI_RX = 8;
constexpr uint8_t SD_SPI_TX = 11;
constexpr uint8_t SD_SPI_SCK = 10;
constexpr uint8_t SD_CS = 9;
constexpr uint8_t PIXEL_PIN = 16;

// Task periods (ms)
constexpr uint32_t READ_DELAY = 5;
constexpr uint32_t SERIAL_DELAY = 100;
constexpr uint32_t REACT_DELAY = 70;
constexpr uint32_t RECOVERY_INTERVAL = 5000;
constexpr uint32_t ERROR_MESSAGE_INTERVAL = 5000;

// MPU6050 registers and scales
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_DATA_START = 0x3B;
constexpr float ACCEL_SCALE = 2048.0f; // +-16g
constexpr float GYRO_SCALE = 16.4f;    // +-2000 deg/s

// Flight status variables. Each transition is an OR of two independent
// conditions (see react()): one on the raw reading, one on its delta.
constexpr float LIFTOFF_ACCEL = 1.9f;
constexpr float LIFTOFF_ACCEL_DELTA = 0.8f;

constexpr float APOGEE_MARGIN_ALTITUDE = 10.0f;
constexpr float APOGEE_ACCEL = 0.4f;
constexpr float APOGEE_ALTITUDE_DELTA = 0.1f;

constexpr float LANDED_MARGIN_ALTITUDE = 5.0f;
constexpr float LANDED_ALTITUDE_DELTA = 0.1f;
constexpr float LANDED_ACCEL = 0.9f;

constexpr bool USE_PARACHUTE = true;

// Number of float values carried by a sample (flags are stored separately).
constexpr size_t READING_COUNT = LOG_FIELD_COUNT;
// How many readings of history are kept to compute deltas. A delta is the
// difference between the newest reading and the one DELTA_WINDOW samples old.
constexpr size_t DELTA_WINDOW = 8;
constexpr size_t DELTA_HISTORY_LEN = DELTA_WINDOW + 1;
// Worst-case size of the readable line produced by formatSerialLine().
constexpr size_t SERIAL_LINE_MAX = 256;
// RAM staging buffer for the SD writer (Tier 2): one write() per batch.
constexpr size_t LOG_BUFFER_SIZE = 4096;
// Flush the buffer at least this often (ms), to bound data loss on power loss.
constexpr uint32_t LOG_FLUSH_INTERVAL = 250;
// Sampling period of writeLog (ms): fast enough for ~100 Hz at 10 ms.
constexpr uint32_t WRITE_LOG_DELAY = 10;

static_assert(sizeof(float) == 4, "binary log requires 32-bit floats");
static_assert(LOG_FIELD_FLAGS_OFFSET < LOG_SAMPLE_PAYLOAD_LEN,
              "flags must fit inside the sample payload");
static_assert(sizeof(log_sample_record) == LOG_SAMPLE_RECORD_LEN,
              "sample record size must match LogFormat.h");
static_assert(sizeof(log_file_header) == LOG_HEADER_RECORD_LEN,
              "header record size must match LogFormat.h");
static_assert(READING_COUNT * sizeof(float) <= LOG_FIELD_FLAGS_OFFSET,
              "readings must fit before the flags byte");

// One sample, in the same order as takeSnapshot() writes it.
// `readings` is what gets logged; `deltas` is derived and RAM-only.
struct SensorSnapshot {
  float readings[READING_COUNT];
  float deltas[READING_COUNT];
  uint8_t status;
};

// FreeRTOS synchronization
SemaphoreHandle_t xMutexSensors = nullptr;
SemaphoreHandle_t xMutexSerial = nullptr;

// BMP280
SoftI2C softI2C(BMP_SDA, BMP_SCL);
BMP280 bmp(softI2C, 0x76);
constexpr float SEA_PRESSURE = 1014.11f;

// NeoPixel
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(1, PIXEL_PIN);

// Shared sensor state (guarded by xMutexSensors)
float temp = 0.0f;
float pressure = 0.0f;
float altitude = 0.0f;
float basealt = 0.0f;
bool basealtSet = false;
float maxalt = 0.0f;
float accel[3] = {0.0f, 0.0f, 0.0f};
float accelTOT = 0.0f;
float gyro[3] = {0.0f, 0.0f, 0.0f};
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
float rot[3] = {0.0f, 0.0f, 0.0f};
uint32_t readCycle_us = 0;
float maxRate_hz = 0.0f;
uint8_t status = LOG_STATUS_IDLE;

// Ring buffer of the last readings, oldest at deltaHead. Guarded by
// xMutexSensors and updated by readSensors() only.
float deltaHistory[DELTA_HISTORY_LEN][READING_COUNT] = {};
size_t deltaHead = 0;
size_t deltaCount = 0;

// Misc state
String logfile;
float beginTime = 0.0f;

// Calculates sea level pressure from a known altitude and pressure
float seaLevelPressure(float alt, float press) {
  return press / powf(1.0f - (alt / 44330.0f), 5.255f);
}

// Reads a big-endian signed 16-bit value.
// The two reads are sequenced explicitly: relying on the evaluation order of
// `(Wire.read() << 8) | Wire.read()` is undefined behaviour in C++.
static int16_t readWire16() {
  const int hi = Wire.read();
  const int lo = Wire.read();
  return static_cast<int16_t>((hi << 8) | (lo & 0xFF));
}

static bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Calibrates the gyroscope. Keep the board still while running.
void cgyro(int samples = 2000) {
  strip.SetPixelColor(0, RgbColor(0, 70, 70));
  strip.Show();
  long sum[3] = {0, 0, 0};

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_DATA_START);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    // Skip accelerometer (3 * 2 bytes) and temperature (2 bytes)
    for (int j = 0; j < 8; j++) {
      Wire.read();
    }

    for (int j = 0; j < 3; j++) {
      sum[j] += readWire16();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  for (int i = 0; i < 3; i++) {
    gyro_offset[i] = (sum[i] / static_cast<float>(samples)) / GYRO_SCALE;
  }
}

void initMPU6050() {
  // Full device reset
  mpuWrite(REG_PWR_MGMT_1, 0x80);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Wake up
  mpuWrite(REG_PWR_MGMT_1, 0x00);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Verify the sensor is alive
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1);

  bool alive = false;
  if (Wire.available()) {
    const uint8_t whoami = Wire.read();
    alive = (whoami == 0x68 || whoami == 0x98);
  }

  if (!alive) {
    if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE) {
      Serial.println("MPU6050 non risponde (WHO_AM_I errato)");
      xSemaphoreGive(xMutexSerial);
    }
    return;
  }

  if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE) {
    Serial.println("MPU6050 recovery OK, ricalibro...");
    xSemaphoreGive(xMutexSerial);
  }

  // +-16g and +-2000 deg/s
  mpuWrite(REG_ACCEL_CONFIG, 0x18);
  vTaskDelay(pdMS_TO_TICKS(10));
  mpuWrite(REG_GYRO_CONFIG, 0x18);
  vTaskDelay(pdMS_TO_TICKS(10));

  cgyro();
}

bool continuity() { return analogRead(PYRO_READ) > 100; }

static const char *statusText(uint8_t value) {
  switch (value) {
    case LOG_STATUS_IDLE:
      return "Idle";
    case LOG_STATUS_FLIGHT:
      return "Flight";
    case LOG_STATUS_DESCENT:
      return "Descent";
    case LOG_STATUS_LANDED:
      return "Landed";
    default:
      return "Unknown";
  }
}

// Per-column variation over DELTA_WINDOW acquisition cycles. Updated by
// readSensors() together with the global state (guarded by xMutexSensors).
float deltas[READING_COUNT] = {};

// Reads one entry of the shared sensor state by its LOG_FIELD_* index.
// Must be called with the sensor mutex held.
static float readingValue(size_t index) {
  switch (index) {
    case LOG_FIELD_ACCEL_X:
      return accel[0];
    case LOG_FIELD_ACCEL_Y:
      return accel[1];
    case LOG_FIELD_ACCEL_Z:
      return accel[2];
    case LOG_FIELD_ACCEL_TOT:
      return accelTOT;
    case LOG_FIELD_GYRO_X:
      return gyro[0];
    case LOG_FIELD_GYRO_Y:
      return gyro[1];
    case LOG_FIELD_GYRO_Z:
      return gyro[2];
    case LOG_FIELD_ROT_X:
      return rot[0];
    case LOG_FIELD_ROT_Y:
      return rot[1];
    case LOG_FIELD_ROT_Z:
      return rot[2];
    case LOG_FIELD_PRESSURE:
      return pressure;
    case LOG_FIELD_ALTITUDE:
      return altitude;
    case LOG_FIELD_REL_ALTITUDE:
      return altitude - basealt;
    case LOG_FIELD_MAX_ALTITUDE:
      return maxalt;
    default:
      return temp;
  }
}

// Copies the shared sensor state into a snapshot. Caller must not hold the mutex.
static void takeSnapshot(SensorSnapshot &snap) {
  if (xSemaphoreTake(xMutexSensors, portMAX_DELAY) != pdTRUE) {
    return;
  }
  for (size_t i = 0; i < READING_COUNT; i++) {
    snap.readings[i] = readingValue(i);
  }

  // RAM-only diagnostics: variation over DELTA_WINDOW acquisition cycles.
  memcpy(snap.deltas, deltas, sizeof(snap.deltas));

  snap.status = status;
  xSemaphoreGive(xMutexSensors);
}

// Recomputes the per-column deltas against the DELTA_WINDOW-samples-old entry.
// Must be called with the sensor mutex held and only by readSensors().
static void updateDeltas() {
  // Store the current sample at deltaHead (the newest slot) ...
  for (size_t i = 0; i < READING_COUNT; i++) {
    deltaHistory[deltaHead][i] = readingValue(i);
  }
  // ... then compare it with the sample DELTA_WINDOW cycles older. Right after
  // boot the history is partially empty, so the oldest known sample is used.
  const size_t reference = (deltaHead + 1) % DELTA_HISTORY_LEN;
  for (size_t i = 0; i < READING_COUNT; i++) {
    deltas[i] = (deltaCount > 0)
                    ? deltaHistory[deltaHead][i] - deltaHistory[reference][i]
                    : 0.0f;
  }
  deltaHead = (deltaHead + 1) % DELTA_HISTORY_LEN;
  if (deltaCount < DELTA_HISTORY_LEN) {
    deltaCount++;
  }
}

// Builds the readable line printed on the serial port (debug only).
static void formatSerialLine(const SensorSnapshot &snap, char *out,
                             size_t outLen) {
  snprintf(out, outLen,
           "%.2f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d | d:%.3f,%.3f,%.3f,%.3f,%.3f",
           (micros() / 1000000.0f) - beginTime, statusText(snap.status),
           snap.readings[LOG_FIELD_ACCEL_X], snap.readings[LOG_FIELD_ACCEL_Y],
           snap.readings[LOG_FIELD_ACCEL_Z], snap.readings[LOG_FIELD_ACCEL_TOT],
           snap.readings[LOG_FIELD_GYRO_X], snap.readings[LOG_FIELD_GYRO_Y],
           snap.readings[LOG_FIELD_GYRO_Z], snap.readings[LOG_FIELD_ROT_X],
           snap.readings[LOG_FIELD_ROT_Y], snap.readings[LOG_FIELD_ROT_Z],
           snap.readings[LOG_FIELD_PRESSURE], snap.readings[LOG_FIELD_ALTITUDE],
           snap.readings[LOG_FIELD_REL_ALTITUDE],
           snap.readings[LOG_FIELD_MAX_ALTITUDE],
           snap.readings[LOG_FIELD_TEMPERATURE], continuity() ? 1 : 0,
           digitalRead(PYRO_OUT) ? 1 : 0,
           // Diagnostic deltas (not logged to SD)
           snap.deltas[LOG_FIELD_ACCEL_TOT], snap.deltas[LOG_FIELD_ALTITUDE],
           snap.deltas[LOG_FIELD_REL_ALTITUDE],
           snap.deltas[LOG_FIELD_PRESSURE],
           snap.deltas[LOG_FIELD_TEMPERATURE]);
}

// Opens the next free "Logs/logN.bin" and writes the file header record.
// Returns true when the file exists and holds a valid header afterwards.
static bool createLogFile(String &path) {
  if (!SD.exists("Logs") && !SD.mkdir("Logs")) {
    return false;
  }

  int fileIndex = 0;
  do {
    path = "Logs/log" + String(fileIndex) + ".bin";
    fileIndex++;
  } while (SD.exists(path));

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    return false;
  }

  log_file_header header;
  memset(&header, 0, sizeof(header));
  header.magic = LOG_MAGIC;
  header.type = LOG_TYPE_HEADER;
  header.size = sizeof(header);
  header.version = LOG_VERSION;
  header.sample_interval_us = WRITE_LOG_DELAY * 1000u;
  strncpy(header.build, "antares-micro-r3", sizeof(header.build) - 1);
  strncpy(header.note, "RP2040 / BMP280 / MPU6050", sizeof(header.note) - 1);

  const size_t written = file.write(reinterpret_cast<const uint8_t *>(&header),
                                    sizeof(header));
  file.flush();
  file.close();
  return written == sizeof(header);
}

// Tasks declarations
void writeLog(void *pvParameters);
void readSensors(void *pvParameters);
void writeSerial(void *pvParameters);
void react(void *pvParameters);

void setup() {
  xMutexSensors = xSemaphoreCreateMutex();
  xMutexSerial = xSemaphoreCreateMutex();

  Serial.begin(115200);
  strip.Begin();
  softI2C.begin();
  delay(200);
  strip.SetPixelColor(0, RgbColor(0, 70, 70));
  strip.Show();
  delay(1800);

  // microSD
  SPI1.setRX(SD_SPI_RX);
  SPI1.setTX(SD_SPI_TX);
  SPI1.setSCK(SD_SPI_SCK);
  if (!SD.begin(SD_CS, SPI1)) {
    Serial.println("MicroSD not working");
    strip.SetPixelColor(0, RgbColor(70, 0, 0));
    strip.Show();
    while (true) {
      delay(10);
    }
  }

  if (createLogFile(logfile)) {
    Serial.println("File creato: " + logfile);
  } else {
    Serial.println("Errore creazione file");
    strip.SetPixelColor(0, RgbColor(70, 0, 0));
    strip.Show();
    while (true) {
      delay(10);
    }
  }

  // BMP280
  if (!bmp.begin()) {
    Serial.println("BMP280 non trovato!");
    strip.SetPixelColor(0, RgbColor(70, 0, 70));
    strip.Show();
    while (true) {
      delay(10);
    }
  }
  Serial.println("BMP280 inizializzato!");

  // Pyro channel
  pinMode(PYRO_OUT, OUTPUT);
  digitalWrite(PYRO_OUT, LOW);

  // MPU6050
  Wire.setSDA(MPU_SDA);
  Wire.setSCL(MPU_SCL);
  Wire.begin();
  Wire.setClock(100000); // slower clock for reliability

  mpuWrite(REG_PWR_MGMT_1, 0x00);
  delay(10);
  mpuWrite(REG_ACCEL_CONFIG, 0x18);
  delay(10);
  mpuWrite(REG_GYRO_CONFIG, 0x18);
  delay(10);

  cgyro();
  Serial.println("Calibrated gyroscope:");
  Serial.print("X offset: ");
  Serial.println(gyro_offset[0]);
  Serial.print("Y offset: ");
  Serial.println(gyro_offset[1]);
  Serial.print("Z offset: ");
  Serial.println(gyro_offset[2]);

  // Diagnostic: MPU6050 WHO_AM_I (address 0x68 or 0x69)
  for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission(false) != 0) {
      continue;
    }
    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
      Serial.print("MPU6050 WHO_AM_I at 0x");
      Serial.print(addr, HEX);
      Serial.print(": 0x");
      Serial.println(Wire.read(), HEX);
    }
  }

  // Diagnostic: BMP280 first reading (uses SoftI2C, not Wire)
  Serial.print("BMP280 diagnostic - Temp: ");
  Serial.print(bmp.readTemperature());
  Serial.print(" C, Pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" hPa");

  beginTime = micros() / 1000000.0f;

  xTaskCreate(readSensors, "Sensors reader", 4096, nullptr, 3, nullptr);
  xTaskCreate(react, "Reactor", 8192, nullptr, 2, nullptr);
  xTaskCreate(writeLog, "Log writer", 8192, nullptr, 1, nullptr);
  xTaskCreate(writeSerial, "Serial writer", 4096, nullptr, 0, nullptr);
}

void loop() {}

void writeLog(void *pvParameters) {
  (void)pvParameters;
  File logFile = SD.open(logfile, FILE_WRITE);
  if (!logFile) {
    if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE) {
      Serial.println("Errore apertura file di log");
      xSemaphoreGive(xMutexSerial);
    }
    vTaskDelete(nullptr);
    return;
  }
  // FILE_WRITE already appends, but seeking to the end makes sure the header
  // written in setup() is never overwritten.
  logFile.seek(logFile.size());

  // RAM staging buffer: records are collected here and written to the SD in
  // one contiguous transfer, which is what turns ~5 Hz into ~100 Hz.
  static uint8_t buffer[LOG_BUFFER_SIZE];
  size_t used = 0;
  TickType_t lastFlush = xTaskGetTickCount();

  for (;;) {
    SensorSnapshot snap;
    takeSnapshot(snap);

    log_sample_record record;
    memset(&record, 0, sizeof(record));
    record.magic = LOG_MAGIC;
    record.type = LOG_TYPE_SAMPLE;
    record.size = sizeof(record);
    record.timestamp_us = micros();
    record.status = snap.status;
    memcpy(record.payload, snap.readings, sizeof(snap.readings));
    uint8_t flags = 0;
    if (continuity()) {
      flags |= LOG_FLAG_CONTINUITY;
    }
    if (digitalRead(PYRO_OUT)) {
      flags |= LOG_FLAG_PYRO;
    }
    // "flags" is the authoritative field; the copy inside the payload keeps the
    // 4-byte alignment of the float block and is only informational.
    record.flags = flags;
    record.payload[LOG_FIELD_FLAGS_OFFSET] = flags;
    // The record CRC is not computed on the MCU (saves flash/time); the
    // converter checks it only when it is non-zero.

    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&record);

    // Flush if the record would not fit, so records are never split.
    if (used + sizeof(record) > sizeof(buffer)) {
      logFile.write(buffer, used);
      logFile.flush();
      used = 0;
      lastFlush = xTaskGetTickCount();
    }
    memcpy(buffer + used, raw, sizeof(record));
    used += sizeof(record);

    // Periodic flush bounds data loss to LOG_FLUSH_INTERVAL on power loss.
    if ((xTaskGetTickCount() - lastFlush) >= pdMS_TO_TICKS(LOG_FLUSH_INTERVAL)) {
      logFile.write(buffer, used);
      logFile.flush();
      used = 0;
      lastFlush = xTaskGetTickCount();
    }

    vTaskDelay(pdMS_TO_TICKS(WRITE_LOG_DELAY));
  }

  logFile.close();
}

void writeSerial(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    SensorSnapshot snap;
    takeSnapshot(snap);

    uint32_t cycle;
    float rate;
    if (xSemaphoreTake(xMutexSensors, portMAX_DELAY) == pdTRUE) {
      cycle = readCycle_us;
      rate = maxRate_hz;
      xSemaphoreGive(xMutexSensors);
    }

    char line[SERIAL_LINE_MAX];
    formatSerialLine(snap, line, sizeof(line));

    if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE) {
      Serial.print("Measurements: ");
      Serial.print(line);
      Serial.print(" | ");
      Serial.print(cycle);
      Serial.print(" us/cycle, ");
      Serial.print(rate, 0);
      Serial.println(" Hz max");
      xSemaphoreGive(xMutexSerial);
    }
    vTaskDelay(pdMS_TO_TICKS(SERIAL_DELAY));
  }
}

void readSensors(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastRecovery = 0;
  TickType_t lastSerialError = 0;
  uint32_t lastMicrosSensor = micros();

  for (;;) {
    const uint32_t cycleStart = micros();

    // BMP280: always read, independently of the MPU6050
    const float localTemp = bmp.readTemperature();
    const float localPressure = bmp.readPressure();
    const float localAltitude =
        44330.0f * (1.0f - powf(localPressure / SEA_PRESSURE, 0.1903f));

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_DATA_START);
    const bool mpuOk = (Wire.endTransmission(false) == 0) &&
                       (Wire.requestFrom(MPU6050_ADDR, 14) >= 14);

    if (!mpuOk) {
      // Keep the last MPU6050 values but still publish fresh BMP280 data
      if (xSemaphoreTake(xMutexSensors, portMAX_DELAY) == pdTRUE) {
        temp = localTemp;
        pressure = localPressure;
        altitude = localAltitude;
        if (!basealtSet) {
          basealt = altitude;
          basealtSet = true;
        }
        if (altitude > maxalt) {
          maxalt = altitude;
        }
        updateDeltas();
        readCycle_us = micros() - cycleStart;
        maxRate_hz = (readCycle_us > 0) ? 1000000.0f / readCycle_us : 0.0f;
        xSemaphoreGive(xMutexSensors);
      }
      lastMicrosSensor = micros();

      const TickType_t now = xTaskGetTickCount();
      if (now - lastSerialError > pdMS_TO_TICKS(ERROR_MESSAGE_INTERVAL)) {
        lastSerialError = now;
        if (xSemaphoreTake(xMutexSerial, portMAX_DELAY) == pdTRUE) {
          Serial.println("Error with MPU6050");
          xSemaphoreGive(xMutexSerial);
        }
      }
      if (now - lastRecovery > pdMS_TO_TICKS(RECOVERY_INTERVAL)) {
        lastRecovery = now;
        initMPU6050();
      }
      vTaskDelay(pdMS_TO_TICKS(READ_DELAY));
      continue;
    }

    int16_t rawAccel[3];
    for (int i = 0; i < 3; i++) {
      rawAccel[i] = readWire16();
    }
    Wire.read(); // temperature high byte (unused)
    Wire.read(); // temperature low byte (unused)

    int16_t rawGyro[3];
    for (int i = 0; i < 3; i++) {
      rawGyro[i] = readWire16();
    }

    const uint32_t nowMicros = micros();
    float dt = (nowMicros - lastMicrosSensor) / 1000000.0f;
    lastMicrosSensor = nowMicros;
    // Avoid a huge integration step after a stall or recovery
    if (dt <= 0.0f || dt > 0.5f) {
      dt = 0.0f;
    }

    if (xSemaphoreTake(xMutexSensors, portMAX_DELAY) == pdTRUE) {
      temp = localTemp;
      pressure = localPressure;
      altitude = localAltitude;
      if (!basealtSet) {
        basealt = altitude;
        basealtSet = true;
      }
      if (altitude > maxalt) {
        maxalt = altitude;
      }

      for (int i = 0; i < 3; i++) {
        accel[i] = rawAccel[i] / ACCEL_SCALE;
      }
      accelTOT = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                       accel[2] * accel[2]);

      for (int i = 0; i < 3; i++) {
        gyro[i] = (rawGyro[i] / GYRO_SCALE) - gyro_offset[i];
        rot[i] += gyro[i] * dt;
        if (rot[i] >= 360.0f) {
          rot[i] -= 360.0f;
        } else if (rot[i] < 0.0f) {
          rot[i] += 360.0f;
        }
      }

      updateDeltas();

      readCycle_us = nowMicros - cycleStart;
      maxRate_hz = (readCycle_us > 0) ? 1000000.0f / readCycle_us : 0.0f;
      xSemaphoreGive(xMutexSensors);
    }

    vTaskDelay(pdMS_TO_TICKS(READ_DELAY));
  }
}

// Transitions the global flight status under the sensor mutex.
static void setStatus(uint8_t newStatus) {
  if (xSemaphoreTake(xMutexSensors, portMAX_DELAY) == pdTRUE) {
    status = newStatus;
    xSemaphoreGive(xMutexSensors);
  }
}

void react(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    SensorSnapshot snap;
    takeSnapshot(snap);

    const float accel = snap.readings[LOG_FIELD_ACCEL_TOT];
    const float alt = snap.readings[LOG_FIELD_ALTITUDE];
    const float relAlt = snap.readings[LOG_FIELD_REL_ALTITUDE];
    const float dAccel = snap.deltas[LOG_FIELD_ACCEL_TOT];
    const float dAlt = snap.deltas[LOG_FIELD_ALTITUDE];

    switch (snap.status) {
      case LOG_STATUS_IDLE:
        // Lift-off: either a strong absolute acceleration (steady state is
        // ~1 g) or a sudden jump compared to a few cycles ago.
        if (accel > LIFTOFF_ACCEL || dAccel > LIFTOFF_ACCEL_DELTA) {
          setStatus(LOG_STATUS_FLIGHT);
        }
        break;

      case LOG_STATUS_FLIGHT:
        // Apogee: altitude well below the maximum seen, or altitude that has
        // started to drop (delta below the noise margin).
        if (alt < snap.readings[LOG_FIELD_MAX_ALTITUDE] - APOGEE_MARGIN_ALTITUDE ||
            dAlt < -APOGEE_ALTITUDE_DELTA) {
          setStatus(LOG_STATUS_DESCENT);
          if(USE_PARACHUTE){
            digitalWrite(PYRO_OUT, HIGH);
          }
        }
        break;

      case LOG_STATUS_DESCENT:
        // Touchdown: near the launch altitude (or altitude no longer changing,
        // delta under LANDED_ALTITUDE_DELTA) and at rest.
        if ((relAlt < LANDED_MARGIN_ALTITUDE || fabsf(dAlt) < LANDED_ALTITUDE_DELTA) &&
            accel < LANDED_ACCEL) {
          setStatus(LOG_STATUS_LANDED);
        }
        break;

      default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(REACT_DELAY));
  }
}
