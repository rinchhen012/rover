/*
 * base.ino - USB <-> nRF24L01 bridge (base station, next to the PC).
 *
 * - Frames arriving on USB are forwarded over the radio (one radio write
 *   per complete frame).
 * - Frames arriving on the radio are streamed out over USB.
 * - PING frames from the PC are answered with ACK right here, so link
 *   latency is measured PC <-> base without involving the rover.
 *
 * Uses a second Arduino (or any SPI-capable board) wired to an nRF24L01:
 *   CE=9, CSN=10, SPI 11/12/13, VCC 3.3V + 100uF cap, GND common.
 */
#include <SPI.h>
#include <RF24.h>
#include "protocol.h"

const uint8_t  PIN_RADIO_CE   = 9;
const uint8_t  PIN_RADIO_CSN  = 10;
const uint16_t RADIO_CHANNEL  = 76;
const uint8_t  ROVER_PIPE[6]  = {0x52, 0x4F, 0x56, 0x45, 0x52};  /* "ROVER" */

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
proto_parser_t serialParser;
uint8_t txBuf[PROTO_FRAME_MAX];
uint8_t rxBuf[PROTO_FRAME_MAX];

void setup() {
  Serial.begin(115200);

  proto_parser_reset(&serialParser);

  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(ROVER_PIPE);
  radio.openReadingPipe(1, ROVER_PIPE);
  radio.startListening();
}

void loop() {
  /* --- PC -> rover --- */
  while (Serial.available()) {
    if (proto_feed(&serialParser, (uint8_t)Serial.read())) {
      proto_frame_t *f = &serialParser.out;
      if (f->type == PROTO_PING) {
        proto_frame_t ack;
        ack.type = PROTO_ACK;
        ack.len = 0;
        size_t n = proto_encode(&ack, txBuf, sizeof(txBuf));
        Serial.write(txBuf, n);   /* instant reply, measures USB+radio RTT */
      } else {
        size_t n = proto_encode(f, txBuf, sizeof(txBuf));
        radio.write(txBuf, n);
      }
    }
  }

  /* --- rover -> PC --- */
  while (radio.available()) {
    uint8_t n = radio.getPayloadSize();
    if (n > PROTO_FRAME_MAX) n = PROTO_FRAME_MAX;
    radio.read(rxBuf, n);
    Serial.write(rxBuf, n);
  }
}
