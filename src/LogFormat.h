#ifndef LogFormat_h
#define LogFormat_h

// Shared binary log format between the firmware and tools/bin2csv.py.
// Keep both in sync: every change here must be mirrored in the converter.
//
// All values are little endian (RP2040 is ARM Cortex-M0+, little endian).
// Every record starts with a 10-byte frame header:
//   uint32 magic    0x47534C41 ("ALSG" little endian)
//   uint16 reserved (0)
//   uint16 type     'H' header, 'S' sample
//   uint16 size     total record size in bytes, frame header included
// Logs are treated as an opaque byte stream: the converter re-syncs on the
// magic value, so truncated or spurious records are skipped instead of
// corrupting the rest of the file.

#include <stddef.h>
#include <stdint.h>

// Frame header
constexpr size_t LOG_FRAME_HEADER_LEN = 10;
constexpr uint32_t LOG_MAGIC = 0x47534C41u; // "ALSG" little endian
constexpr uint16_t LOG_TYPE_HEADER = 0x0048; // 'H'
constexpr uint16_t LOG_TYPE_SAMPLE = 0x0053; // 'S'
constexpr uint8_t LOG_VERSION = 1;

// Status values stored in log_sample_record::status.
constexpr uint8_t LOG_STATUS_IDLE = 0;
constexpr uint8_t LOG_STATUS_FLIGHT = 1;
constexpr uint8_t LOG_STATUS_DESCENT = 2;
constexpr uint8_t LOG_STATUS_LANDED = 3;

constexpr size_t LOG_TEXT_LEN = 32;

// Sample payload: 15 float values (60 bytes) + 1 byte of flags + 3 padding.
constexpr size_t LOG_FIELD_COUNT = 15;
constexpr size_t LOG_SAMPLE_PAYLOAD_LEN = 64;
constexpr size_t LOG_FIELD_FLAGS_OFFSET = LOG_FIELD_COUNT * sizeof(float);
constexpr uint8_t LOG_FLAG_CONTINUITY = 0x01;
constexpr uint8_t LOG_FLAG_PYRO = 0x02;

// Field order inside the payload, expressed as float indices.
constexpr size_t LOG_FIELD_ACCEL_X = 0;
constexpr size_t LOG_FIELD_ACCEL_Y = 1;
constexpr size_t LOG_FIELD_ACCEL_Z = 2;
constexpr size_t LOG_FIELD_ACCEL_TOT = 3;
constexpr size_t LOG_FIELD_GYRO_X = 4;
constexpr size_t LOG_FIELD_GYRO_Y = 5;
constexpr size_t LOG_FIELD_GYRO_Z = 6;
constexpr size_t LOG_FIELD_ROT_X = 7;
constexpr size_t LOG_FIELD_ROT_Y = 8;
constexpr size_t LOG_FIELD_ROT_Z = 9;
constexpr size_t LOG_FIELD_PRESSURE = 10;
constexpr size_t LOG_FIELD_ALTITUDE = 11;
constexpr size_t LOG_FIELD_REL_ALTITUDE = 12;
constexpr size_t LOG_FIELD_MAX_ALTITUDE = 13;
constexpr size_t LOG_FIELD_TEMPERATURE = 14;

// Header record, written once at the start of every log file.
// Binary layout (packed, 81 bytes):
//   frame header (10) | version | reserved | sample_interval_us(4) |
//   reserved2 | build[32] | note[32]
struct __attribute__((packed)) log_file_header {
  uint32_t magic;
  uint16_t reserved;
  uint16_t type;
  uint16_t size;
  uint8_t version;
  uint8_t reserved2;
  uint32_t sample_interval_us;
  uint8_t reserved3;
  char build[LOG_TEXT_LEN];
  char note[LOG_TEXT_LEN];
};

// Sample record, written for every acquisition cycle.
// Binary layout (packed, 86 bytes):
//   frame header (10) | timestamp_us | status | flags | crc32 | payload[64]
struct __attribute__((packed)) log_sample_record {
  uint32_t magic;
  uint16_t reserved;
  uint16_t type;
  uint16_t size;
  uint32_t timestamp_us;
  uint8_t status;
  uint8_t flags;
  uint32_t crc32; // 0 = not computed (current firmware)
  uint8_t payload[LOG_SAMPLE_PAYLOAD_LEN];
};

constexpr size_t LOG_HEADER_RECORD_LEN = sizeof(log_file_header);
constexpr size_t LOG_SAMPLE_RECORD_LEN = sizeof(log_sample_record);

// Layout guards: tools/bin2csv.py mirrors these exact sizes.
static_assert(LOG_FRAME_HEADER_LEN == 10, "frame header layout changed");
static_assert(LOG_HEADER_RECORD_LEN == 10 + 7 + 2 * LOG_TEXT_LEN,
              "header record layout changed");
static_assert(LOG_SAMPLE_RECORD_LEN ==
                  10 + 4 + 1 + 1 + 4 + LOG_SAMPLE_PAYLOAD_LEN,
              "sample record layout changed");

#endif
