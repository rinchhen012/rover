/* C-side parity check for the shared protocol header.
 * Compiled and executed by tests/test_protocol.py to prove the Python
 * mirror and the C header produce identical bytes and parsing behavior.
 * Build: cc -I../shared c_parity.c -o c_parity
 */
#include <stdio.h>
#include <string.h>
#include "protocol.h"

static void print_hex(const uint8_t *buf, size_t n) {
  for (size_t i = 0; i < n; i++) printf("%02x", buf[i]);
  printf("\n");
}

int main(void) {
  uint8_t out[PROTO_FRAME_MAX];
  proto_frame_t f;

  /* 1. CRC vector */
  const uint8_t v[] = {0xAA, 0x01, 0x04};
  printf("crc:%02x\n", proto_crc8(0, v, sizeof(v)));

  /* 2. CMD_DRIVE frame for throttle=123, steer=-45 */
  f.type = PROTO_CMD_DRIVE;
  f.len = 4;
  proto_put_i16(f.payload, 123);
  proto_put_i16(f.payload + 2, -45);
  print_hex(out, proto_encode(&f, out, sizeof(out)));

  /* 3. TELEM frame */
  f.type = PROTO_TELEM;
  f.len = 6;
  proto_put_u16(f.payload, 155);
  proto_put_u16(f.payload + 2, 7400);
  f.payload[4] = 33;
  f.payload[5] = 1;
  print_hex(out, proto_encode(&f, out, sizeof(out)));

  /* 4. Parser: valid frame, garbage, valid frame */
  {
    proto_parser_t p;
    proto_parser_reset(&p);
    uint8_t good[PROTO_FRAME_MAX];
    size_t gn = proto_encode(&f, good, sizeof(good));
    const uint8_t garbage[] = {0x55, 0x00, 0xAA, 0xFF, 0x12};
    for (size_t i = 0; i < gn; i++) proto_feed(&p, good[i]);
    for (size_t i = 0; i < sizeof(garbage); i++) proto_feed(&p, garbage[i]);
    for (size_t i = 0; i < gn; i++) proto_feed(&p, good[i]);
    printf("parse:%u:%u:%u:%u\n", p.frames_ok, p.frames_bad,
           p.out.type, p.out.payload[4]);
  }

  /* 5. Payload endianness round trip */
  {
    uint8_t b[2];
    proto_put_u16(b, 0x1234);
    printf("endian:%u\n", (unsigned)proto_get_u16(b));
  }
  return 0;
}
