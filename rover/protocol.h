/*
 * rover protocol.h - shared wire protocol for the rover project.
 *
 * Canonical copy lives in rover/shared/protocol.h and is copied verbatim
 * into every sketch folder (rover/, base/, esp32cam/). Edit this file,
 * then re-copy. The Python mirror lives in gui/protocol.py.
 *
 * Frame layout:
 *   [0] SYNC 0xAA
 *   [1] TYPE
 *   [2] LEN  (payload length, max PROTO_MAX_PAYLOAD)
 *   [3..3+LEN-1] PAYLOAD
 *   [3+LEN] CRC8 over bytes [0 .. 3+LEN-1]  (poly 0x07, init 0x00)
 *
 * Frame types:
 *   CMD_DRIVE  payload: i16 throttle (-255..255), i16 steer (-255..255)
 *   CMD_GIMBAL payload: u8 pan (0..180), u8 tilt (0..180)
 *   TELEM      payload: u16 dist_cm, u16 batt_mV, u8 loop_ms, u8 radio_ok
 *                        (radio_ok: 1 = carrier detected (RPD), 0 = none)
 *   PING       no payload -> receiver answers ACK
 *   ACK        no payload
 */
#ifndef ROVER_PROTOCOL_H
#define ROVER_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define PROTO_SYNC          0xAA
#define PROTO_MAX_PAYLOAD   16
#define PROTO_FRAME_MAX     (PROTO_MAX_PAYLOAD + 4)

typedef enum {
  PROTO_CMD_DRIVE = 0x01,
  PROTO_CMD_GIMBAL = 0x02,
  PROTO_TELEM = 0x03,
  PROTO_PING = 0x04,
  PROTO_ACK = 0x05
} proto_type_t;

typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

static inline uint8_t proto_crc8(uint8_t crc, const uint8_t *data, size_t n) {
  for (size_t i = 0; i < n; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

/* Encodes a frame into out[]. Returns frame length, 0 on error. */
static inline size_t proto_encode(const proto_frame_t *f, uint8_t *out, size_t cap) {
  if (f->len > PROTO_MAX_PAYLOAD || cap < (size_t)f->len + 4) return 0;
  out[0] = PROTO_SYNC;
  out[1] = f->type;
  out[2] = f->len;
  memcpy(out + 3, f->payload, f->len);
  out[3 + f->len] = proto_crc8(0, out, 3 + f->len);
  return (size_t)f->len + 4;
}

typedef enum {
  P_WAIT_SYNC,
  P_WAIT_TYPE,
  P_WAIT_LEN,
  P_WAIT_PAYLOAD,
  P_WAIT_CRC
} proto_parse_state_t;

typedef struct {
  proto_parse_state_t state;
  uint8_t frame[PROTO_FRAME_MAX];
  uint8_t idx;
  proto_frame_t out;
  uint16_t frames_ok;
  uint16_t frames_bad;
} proto_parser_t;

static inline void proto_parser_reset(proto_parser_t *p) {
  p->state = P_WAIT_SYNC;
  p->idx = 0;
  p->frames_ok = 0;
  p->frames_bad = 0;
}

/* Feed one byte. Returns 1 when a complete valid frame is ready in p->out. */
static inline int proto_feed(proto_parser_t *p, uint8_t b) {
  switch (p->state) {
    case P_WAIT_SYNC:
      if (b == PROTO_SYNC) {
        p->frame[0] = b;
        p->state = P_WAIT_TYPE;
      }
      break;
    case P_WAIT_TYPE:
      p->frame[1] = b;
      p->state = P_WAIT_LEN;
      break;
    case P_WAIT_LEN:
      p->frame[2] = b;
      if (b > PROTO_MAX_PAYLOAD) {           /* nonsense length: resync */
        p->frames_bad++;
        p->state = P_WAIT_SYNC;
      } else {
        p->idx = 3;
        p->state = P_WAIT_PAYLOAD;
      }
      break;
    case P_WAIT_PAYLOAD:
      p->frame[p->idx++] = b;
      if (p->idx == 3 + p->frame[2]) p->state = P_WAIT_CRC;
      break;
    case P_WAIT_CRC:
      if (proto_crc8(0, p->frame, 3 + p->frame[2]) == b) {
        p->out.type = p->frame[1];
        p->out.len = p->frame[2];
        memcpy(p->out.payload, p->frame + 3, p->out.len);
        p->frames_ok++;
        p->state = P_WAIT_SYNC;
        return 1;
      }
      p->frames_bad++;
      p->state = P_WAIT_SYNC;
      if (b == PROTO_SYNC) {                 /* maybe a frame starts right here */
        p->frame[0] = b;
        p->state = P_WAIT_TYPE;
      }
      break;
  }
  return 0;
}

/* ---- payload helpers (big-endian) ---- */
static inline void proto_put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}
static inline uint16_t proto_get_u16(const uint8_t *p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}
static inline void proto_put_i16(uint8_t *p, int16_t v) {
  proto_put_u16(p, (uint16_t)v);
}
static inline int16_t proto_get_i16(const uint8_t *p) {
  return (int16_t)proto_get_u16(p);
}

#endif /* ROVER_PROTOCOL_H */
