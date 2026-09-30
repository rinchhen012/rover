# Rover — pan/tilt camera + PC & phone teleoperation

A 4WD rover you drive from your PC keyboard **and** your phone, with a
live camera, gimbal, obstacle sensor, and battery telemetry — over two
independent wireless links.

```
[PC keyboard] ─USB─> [Base Uno] ──nRF24L01 radio──> ─┐
                                                      ├─> [Rover Uno] ──> L298N + 4 motors
[Phone browser] ──WiFi──> [ESP32-CAM] ──UART──> ─────┘      ├─> pan/tilt gimbal + camera
                                                            ├─> HC-SR04, battery divider
                                                            └─> failsafe: stop after 500ms w/o cmd
```

Why two links: the nRF24 radio gives the PC low-latency control and keeps
working if the ESP32's WiFi dies; the phone gets video + joystick from the
ESP32-CAM. The rover's Uno arbitrates — **last fresh packet wins** from
either source.

## Files

| Path | Role |
|---|---|
| `rover/rover.ino` | vehicle firmware: drive ramp, gimbal smoothing, sensors, failsafe, arbitration |
| `base/base.ino` | USB ↔ nRF24 bridge at the PC |
| `esp32cam/esp32cam.ino` | camera + web server + phone→rover command bridge |
| `esp32cam/index.html` | readable copy of the embedded phone UI (keep in sync) |
| `shared/protocol.h` | wire protocol (copied verbatim into each sketch dir) |
| `gui/rover_gui.py` | PC control + live dashboard (has `--sim` mode) |
| `gui/protocol.py` | Python mirror of the protocol |
| `tests/` | protocol tests incl. C↔Python parity check |

## Parts (~$40 beyond the starter kit)

| Part | Qty | Notes |
|---|---|---|
| 4WD robot car chassis + motors + wheels | 1 | generic "smart car chassis" |
| L298N motor driver | 1 | dual H-bridge |
| SG90 9g servo | 2 | pan/tilt gimbal |
| ESP32-CAM (AI Thinker) | 1 | + FTDI programmer to flash it |
| nRF24L01+ PA/LNA (antenna version) | 2 | one rover, one base |
| Arduino Uno (clone fine) | 1 | base station |
| 18650 holder (2×) + 2× 18650 | 1 | ~7.4V pack for motors |
| 100 µF capacitor | 2 | across each nRF24 VCC/GND (mandatory) |
| 10 kΩ + 4.7 kΩ resistors | 2 | battery divider |
| 1 kΩ resistor | 1 | ESP32 TX → Uno RX safety |

## Wiring

### Rover Uno

| Rover Uno pin | Goes to |
|---|---|
| 2 | nRF24 CE |
| 10 | nRF24 CSN |
| 11, 12, 13 | nRF24 MOSI, MISO, SCK |
| 3.3 V | nRF24 VCC **with 100 µF across VCC/GND** |
| 6 (ENA), 5 (ENB) | L298N enable A (left), enable B (right) |
| 7 (IN1), 8 (IN2) | L298N inputs → left motor pair |
| 4 (IN3), A0 (IN4) | L298N inputs → right motor pair |
| 9 | pan servo signal (servo power from L298N 5 V) |
| 3 | tilt servo signal |
| A1, A2 | HC-SR04 TRIG, ECHO |
| A3 | battery divider midpoint (10 kΩ → battery+, 4.7 kΩ → GND) |
| 0 (RX) | ESP32-CAM TX via 1 kΩ |
| 1 (TX) | ESP32-CAM RX |
| GND | common ground with L298N, ESP32, battery |

L298N: motor power from 18650 pack (12 V input terminal), 5 V output
powers the Uno (via Vin) and the servos.

### Base Uno (second Uno, next to the PC)

| Base Uno pin | Goes to |
|---|---|
| 9 | nRF24 CE (free pin on this board) |
| 10 | nRF24 CSN |
| 11, 12, 13 | nRF24 MOSI, MISO, SCK |
| 3.3 V | nRF24 VCC with 100 µF cap |
| USB | to PC |

### ESP32-CAM

| ESP32-CAM | Goes to |
|---|---|
| U0TXD (GPIO1) | Uno pin 0 via 1 kΩ |
| U0RXD (GPIO3) | Uno pin 1 |
| 5 V / GND | L298N 5 V out / ground |

Power note: the ESP32-CAM draws ~300 mA with WiFi. If the L298N's 5 V
regulator sags, power the ESP32-CAM from a separate 5 V UBEC or power bank.

## Protocol

```
[0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC8]      CRC8: poly 0x07, init 0
```

| TYPE | Payload |
|---|---|
| `CMD_DRIVE` (0x01) | i16 throttle, i16 steer (−255..255) |
| `CMD_GIMBAL` (0x02) | u8 pan, u8 tilt (0..180) |
| `TELEM` (0x03) | u16 dist_cm, u16 batt_mV, u8 loop_ms, u8 radio_ok |
| `PING` (0x04) | — |
| `ACK` (0x05) | — |

Corrupted or partial frames are detected by CRC8 and the receiver
re-syncs on the next `0xAA`. `PING`/`ACK` measures link latency (the base
answers pings itself, so RTT ≈ USB + radio round trip).

## Arbitration & failsafe

- The rover accepts `CMD_DRIVE`/`CMD_GIMBAL` from **either** the radio
  (PC) or UART (phone), whichever arrives last.
- If no command arrives from anywhere for **500 ms**, the rover stops the
  motors and holds the gimbal (edit `FAILSAFE_MS` in `rover.ino`).
- Throttle is ramped at `DRIVE_STEP` per 25 ms tick for smooth starts;
  gimbal moves at `GIMBAL_STEP` per tick.

## Phase-by-phase checkout (do after wiring, in order)

1. **Radio link** — flash both Unos, run `gui/rover_gui.py --sim`? No:
   real mode with no camera: `gui/rover_gui.py --port <port> --camera none`.
   Watch `LATENCY` climb and fall; move the base away to see the link degrade.
2. **Drive** — jack the wheels up first. W/S throttle, A/D steering,
   SPACE stop. Verify failsafe: lift wheels, hold W, unplug the base → motors
   stop ≤ 500 ms. Flip `LEFT_REV`/`RIGHT_REV` if a side runs backwards.
3. **Gimbal + camera** — arrows pan/tilt the camera; open the ESP32-CAM
   page in a phone browser (SSID `RoverCam`, password `rover1234`), video
   streams at `/stream`.
4. **Phone control** — drive from the phone joystick; then drive from the
   PC; both should work (last input wins).
5. **Telemetry** — distance/battery/loop/radio gauges update in the GUI
   and on the phone chips.
6. **Outside** — full drive test at 20 m+ line of sight; tune
   `DRIVE_STEP`, `GIMBAL_STEP`, camera quality.

## Troubleshooting

| Symptom | Fix |
|---|---|
| No radio link, both sides silent | 100 µF cap across nRF24 VCC/GND; check 3.3 V supply; same channel on both sketches |
| Radio works at 1 m but not 5 m | Try `RF24_2MBPS`, check antenna solder, keep antennas vertical |
| Motors twitch but don't run | L298N 5 V jumper in place, battery pack ≥ 7 V |
| Servo buzzes / browns out | power servos from L298N 5 V, not Uno 5 V |
| ESP32-CAM won't flash | hold GPIO0 low through the FTDI cable; disconnect Uno UART during flashing |
| Phone page loads, no video | check PSRAM enabled in FQBN (see SETUP.md); camera module seated |
| GUI: "NO VIDEO" | confirm ESP32-CAM on `http://192.168.4.1:81/stream` in a browser first |

## Tuning constants (rover.ino)

`DRIVE_STEP` (accel), `GIMBAL_STEP` (smoothness), `FAILSAFE_MS` (safety),
`SONAR_MAX_CM`, `BATT_RATIO` (match your divider), `LEFT_REV`/`RIGHT_REV`
(motor polarity).
