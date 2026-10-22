// ===== Llama Robot: shared sensor telemetry packet =====
//
// One packet type for every sensor on the robot. A sender fills in its own name, how
// many values it is reporting and the values themselves; the bridge prints whatever
// arrives without needing to know the sensor exists.
//
// This replaces the older scheme where a packet's TYPE was its LENGTH (Drive = 4,
// Encoder = 12, Imu = 16). That worked only while every sensor had a distinct size,
// so two boards of the same kind were indistinguishable and every new sensor had to
// be padded to a unique length. Here the name identifies the sender, so length is
// free -- and because the name travels in the packet, there is no id table to keep in
// sync between the firmware and the PC.
//
// DrivePacket is deliberately NOT part of this. The motor board and the stick
// controller keep their own 4-byte struct and need no reflash; the bridge tells a
// drive command from telemetry by length, which still works since 4 != 41.
//
// Wire format (41 bytes, packed, little-endian -- all ESP32s here agree):
//   char     name[12]   NUL-terminated short name, e.g. "TOF_FRONT"
//   uint8_t  count      how many of values[] are meaningful (0..SENSOR_MAX_VALUES)
//   uint32_t ms         sender's millis() at send time; the PC uses it for dt
//   int32_t  values[6]  the readings, meaning defined by the sending sketch
//
// int32 for everything is wasteful for an IMU's int16s, but a packet costs ~1 ms of
// airtime at the ESP-NOW broadcast base rate almost regardless of its size -- the
// framing and preamble dominate, so 41 bytes and 17 bytes cost nearly the same. Fewer
// packets is what saves airtime, not smaller ones, which is why a board with several
// sensors should bundle them into one packet rather than send one each.
//
// Installing: this folder is an Arduino library. Symlink (or copy) it into your
// sketchbook so `#include <LlamaSensors.h>` resolves:
//   ln -s <repo>/firmware/libraries/LlamaSensors ~/Arduino/libraries/LlamaSensors

#pragma once

#include <stdint.h>
#include <string.h>

#define SENSOR_NAME_LEN 12
#define SENSOR_MAX_VALUES 6

typedef struct __attribute__((packed)) {
  char name[SENSOR_NAME_LEN];
  uint8_t count;
  uint32_t ms;
  int32_t values[SENSOR_MAX_VALUES];
} SensorPacket;

// A reading the sender could not trust. Travels as a value so the PC can see that the
// sensor reported and was simply unable to measure, which is different from silence.
// Use this only where -1 is not a physically possible reading -- a distance in mm, say.
#define SENSOR_VALUE_INVALID ((int32_t)-1)

// The same idea for sensors whose natural range DOES include -1, such as an encoder's
// cumulative count, which legitimately goes negative one tick at a time. INT32_MIN is
// reachable in principle but only after days of continuous one-way rotation, by which
// point the counter would be wrapping anyway.
#define SENSOR_VALUE_UNAVAILABLE ((int32_t)(-2147483647 - 1))

// Fill a packet safely: truncates an over-long name, always NUL-terminates, clamps the
// count, and zeroes the unused value slots so nothing stale is transmitted.
static inline void sensorPacketInit(SensorPacket* p, const char* name,
                                    uint8_t count, uint32_t ms) {
  memset(p, 0, sizeof(*p));
  strncpy(p->name, name, SENSOR_NAME_LEN - 1);  // memset already left the last byte NUL
  p->count = (count > SENSOR_MAX_VALUES) ? SENSOR_MAX_VALUES : count;
  p->ms = ms;
}

// Does this look like one of our packets at all? The bridge hears every ESP-NOW
// broadcast on the channel, including traffic from unrelated devices, and a foreign
// frame that happens to be sizeof(SensorPacket) long would otherwise be printed as a
// sensor reading with a garbage name and nonsense values. A real packet has a non-empty
// name made only of [A-Za-z0-9_] (which is all a sender can produce), is NUL-terminated
// within the field, and reports no more values than the packet can hold.
static inline bool sensorPacketPlausible(const SensorPacket* p) {
  if (p->count > SENSOR_MAX_VALUES) return false;
  bool terminated = false;
  for (uint8_t i = 0; i < SENSOR_NAME_LEN; i++) {
    char c = p->name[i];
    if (c == '\0') { terminated = (i > 0); break; }   // i == 0 means an empty name
    bool allowed = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                   (c >= '0' && c <= '9') || c == '_';
    if (!allowed) return false;
  }
  return terminated;
}

// Make a received name safe to print on a whitespace-delimited serial line. A name from
// a half-flashed or mis-wired board can contain spaces, newlines or junk, which would
// corrupt the line and desync the PC-side parser -- so anything outside [A-Za-z0-9_] is
// replaced and termination is forced. dst must hold SENSOR_NAME_LEN bytes.
static inline void sensorPacketSafeName(const SensorPacket* p, char* dst) {
  for (uint8_t i = 0; i < SENSOR_NAME_LEN - 1; i++) {
    char c = p->name[i];
    if (c == '\0') { dst[i] = '\0'; return; }
    bool printable = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                     (c >= '0' && c <= '9') || c == '_';
    dst[i] = printable ? c : '_';
  }
  dst[SENSOR_NAME_LEN - 1] = '\0';
}
