/*
 * rover.ino - Rover vehicle firmware.
 *
 * Sits on the rover. Receives CMD_DRIVE / CMD_GIMBAL from TWO sources:
 *   - PC     -> base Uno -> nRF24L01 radio  (SPI, CE=2 CSN=10, 11/12/13)
 *   - phone  -> ESP32-CAM -> UART           (hardware Serial, pins 0/1)
 * Last-fresh-packet-wins; hard-stop failsafe if nothing arrives in 500ms.
 *
 * Sends TELEM (distance, battery, loop time, radio_ok) on BOTH links every
 * 100ms so the PC GUI and the phone web page both see live data.
 *
 * See README.md for wiring tables. protocol.h is shared with the other
 * boards and with gui/protocol.py - keep them in sync.
 */
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <NewPing.h>
#include "protocol.h"

/* ---------------- pin map ---------------- */
const uint8_t PIN_RADIO_CE   = 2;
const uint8_t PIN_RADIO_CSN  = 10;   /* SPI: 11 MOSI, 12 MISO, 13 SCK */
const uint8_t PIN_ENA        = 6;    /* L298N enable A -> LEFT motors  */
const uint8_t PIN_ENB        = 5;    /* L298N enable B -> RIGHT motors */
const uint8_t PIN_IN1        = 7;
const uint8_t PIN_IN2        = 8;
const uint8_t PIN_IN3        = 4;
const uint8_t PIN_IN4        = A0;
const uint8_t PIN_SERVO_PAN  = 9;
const uint8_t PIN_SERVO_TILT = 3;
const uint8_t PIN_SONAR_TRIG = A1;
const uint8_t PIN_SONAR_ECHO = A2;
const uint8_t PIN_BATT       = A3;

/* ---------------- radio ---------------- */
const uint16_t RADIO_CHANNEL = 76;
const uint8_t  ROVER_PIPE[6] = {0x52, 0x4F, 0x56, 0x45, 0x52};  /* "ROVER" */

/* ---------------- tuning ---------------- */
const bool     LEFT_REV      = false;  /* flip to correct a side that */
const bool     RIGHT_REV     = false;  /* drives backwards            */
const uint8_t  MAX_SPEED     = 255;
const uint8_t  DRIVE_STEP    = 16;     /* throttle ramp per 25ms tick */
const uint16_t FAILSAFE_MS   = 500;    /* stop when no cmd for this   */
const uint8_t  GIMBAL_STEP   = 2;      /* deg per 25ms tick           */
const uint16_t SONAR_MAX_CM  = 400;
const uint16_t TELEM_MS      = 100;    /* telemetry interval          */
const uint8_t  LOOP_TICK_MS  = 25;     /* drive/gimbal control tick   */
const float    BATT_RATIO    = 3.13f;  /* 10k/4.7k divider            */

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
Servo servoPan, servoTilt;
NewPing sonar(PIN_SONAR_TRIG, PIN_SONAR_ECHO, SONAR_MAX_CM);

proto_parser_t radioParser;
proto_parser_t serialParser;
uint8_t radioBuf[PROTO_FRAME_MAX];
uint8_t txBuf[PROTO_FRAME_MAX];

int16_t thrTarget = 0, steerTarget = 0;
int16_t thrCur = 0, steerCur = 0;
uint8_t panTarget = 90, tiltTarget = 90;
uint8_t panCur = 90, tiltCur = 90;

unsigned long lastCmdMs = 0;
unsigned long lastTickMs = 0;
unsigned long lastTelemMs = 0;
uint8_t loopMs = 0;    /* EMA of loop duration */
uint8_t radioOk = 0;   /* carrier-detect flag  */

void setMotor(uint8_t inA, uint8_t inB, uint8_t en, int16_t v, bool rev) {
  if (rev) v = (int16_t)(-v);
  if (v > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else if (v < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else            { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }
  analogWrite(en, (uint8_t)abs(v));
}

void driveMotors(int16_t throttle, int16_t steer) {
  int16_t left  = throttle + steer;   /* +steer = turn right */
  int16_t right = throttle - steer;
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, left,  LEFT_REV);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, right, RIGHT_REV);
}

void sendFrame(const proto_frame_t *f, bool onRadio) {
  size_t n = proto_encode(f, txBuf, sizeof(txBuf));
  if (n == 0) return;
  if (onRadio) radio.write(txBuf, n);
  else         Serial.write(txBuf, n);
}

void sendAck(bool onRadio) {
  proto_frame_t ack;
  ack.type = PROTO_ACK;
  ack.len = 0;
  sendFrame(&ack, onRadio);
}

void onFrame(const proto_frame_t *f) {
  switch (f->type) {
    case PROTO_CMD_DRIVE:
      thrTarget = constrain(proto_get_i16(f->payload), -MAX_SPEED, MAX_SPEED);
      steerTarget = constrain(proto_get_i16(f->payload + 2), -MAX_SPEED, MAX_SPEED);
      lastCmdMs = millis();
      break;
    case PROTO_CMD_GIMBAL:
      panTarget = constrain(f->payload[0], 0, 180);
      tiltTarget = constrain(f->payload[1], 0, 180);
      lastCmdMs = millis();
      break;
    case PROTO_PING:
      sendAck(false);
      break;
    default:
      break;   /* TELEM / ACK are not commands */
  }
}

uint16_t readBatteryMv(void) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_BATT);
  float v = (sum / 8.0f) * (5.0f / 1023.0f) * BATT_RATIO;
  return (uint16_t)(v * 1000.0f);
}

void sendTelemetry(void) {
  proto_frame_t f;
  f.type = PROTO_TELEM;
  f.len = 6;

  uint16_t dist = sonar.ping_cm();
  if (dist == 0 || dist > SONAR_MAX_CM) dist = 0;

  proto_put_u16(f.payload, dist);
  proto_put_u16(f.payload + 2, readBatteryMv());
  f.payload[4] = loopMs;
  f.payload[5] = radioOk;

  sendFrame(&f, true);   /* to the PC   */
  sendFrame(&f, false);  /* to the phone */
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);

  servoPan.attach(PIN_SERVO_PAN);
  servoPan.write(panCur);
  servoTilt.attach(PIN_SERVO_TILT);
  servoTilt.write(tiltCur);

  proto_parser_reset(&radioParser);
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
  unsigned long now = millis();
  unsigned long t0 = micros();

  /* --- radio RX (PC link) --- */
  uint8_t pipe = 0;
  while (radio.available(&pipe)) {
    uint8_t n = radio.getPayloadSize();
    if (n > PROTO_FRAME_MAX) n = PROTO_FRAME_MAX;
    radio.read(radioBuf, n);
    for (uint8_t i = 0; i < n; i++) {
      if (proto_feed(&radioParser, radioBuf[i])) onFrame(&radioParser.out);
    }
    radioOk = radio.testRPD() ? 1 : 0;
  }

  /* --- serial RX (phone link via ESP32-CAM) --- */
  while (Serial.available()) {
    if (proto_feed(&serialParser, (uint8_t)Serial.read())) onFrame(&serialParser.out);
  }

  /* --- failsafe: no fresh command from anywhere -> stop --- */
  if (now - lastCmdMs > FAILSAFE_MS) {
    thrTarget = 0;
    steerTarget = 0;
  }

  /* --- control tick: ramp throttle, move gimbal --- */
  if (now - lastTickMs >= LOOP_TICK_MS) {
    lastTickMs = now;

    if (thrCur < thrTarget) thrCur = min(thrTarget, (int16_t)(thrCur + DRIVE_STEP));
    else if (thrCur > thrTarget) thrCur = max(thrTarget, (int16_t)(thrCur - DRIVE_STEP));
    if (steerCur < steerTarget) steerCur = min(steerTarget, (int16_t)(steerCur + DRIVE_STEP));
    else if (steerCur > steerTarget) steerCur = max(steerTarget, (int16_t)(steerCur - DRIVE_STEP));

    driveMotors(thrCur, steerCur);

    if (panCur < panTarget) panCur = min(panTarget, (uint8_t)(panCur + GIMBAL_STEP));
    else if (panCur > panTarget) panCur = max(panTarget, (uint8_t)(panCur - GIMBAL_STEP));
    if (tiltCur < tiltTarget) tiltCur = min(tiltTarget, (uint8_t)(tiltCur + GIMBAL_STEP));
    else if (tiltCur > tiltTarget) tiltCur = max(tiltTarget, (uint8_t)(tiltCur - GIMBAL_STEP));

    servoPan.write(panCur);
    servoTilt.write(tiltCur);
  }

  /* --- telemetry --- */
  if (now - lastTelemMs >= TELEM_MS) {
    lastTelemMs = now;
    sendTelemetry();
  }

  uint32_t dt = (uint32_t)(micros() - t0);
  if (dt < 255) loopMs = (uint8_t)((loopMs * 7 + dt / 1000) / 8);
}
