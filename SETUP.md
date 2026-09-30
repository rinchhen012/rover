# Setup — one-time machine prep

## 1. Toolchain (macOS)

```sh
brew install arduino-cli
```

Already done on this Mac. Cores and libraries:

```sh
arduino-cli core install arduino:avr
arduino-cli core install esp32:esp32
arduino-cli lib install "RF24" "NewPing" "Servo"
```

The ESP32 camera driver ships prebuilt inside the esp32 core
(3.3.11) — no extra library needed. **Do not** install a separate
esp32-camera library; it will shadow the core's and break the build.

## 2. Python environment

```sh
python3 -m venv rover/.venv
rover/.venv/bin/pip install pyserial pygame opencv-python-headless
```

`opencv-python-headless` (not `opencv-python`) on purpose: the regular
wheel bundles SDL2 and conflicts with pygame on macOS.

## 3. Compile / flash the three boards

```sh
# vehicle + base (Arduino Uno):
arduino-cli compile -b arduino:avr:uno rover/rover
arduino-cli upload -b arduino:avr:uno -p /dev/cu.usbmodemXXXX rover/rover

arduino-cli compile -b arduino:avr:uno rover/base
arduino-cli upload -b arduino:avr:uno -p /dev/cu.usbmodemYYYY rover/base

# camera board (ESP32-CAM via FTDI, GPIO0 held low to flash):
arduino-cli compile -b esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=default,FlashSize=4M rover/esp32cam
arduino-cli upload -b esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=default,FlashSize=4M -p /dev/cu.usbserial-XXXX rover/esp32cam
```

Note: when flashing the rover Uno, the ESP32-CAM may be attached to pins
0/1 — the bootloader ignores them, so it usually works; if upload fails,
unplug the ESP32 UART wires first.

## 4. Run the PC GUI

```sh
# hardware mode (base Uno plugged in):
rover/.venv/bin/python rover/gui/rover_gui.py --port /dev/cu.usbmodemXXXX

# sim mode (no hardware at all):
rover/.venv/bin/python rover/gui/rover_gui.py --sim
```

## 5. Phone control

1. Power the rover (ESP32-CAM boots, WiFi AP `RoverCam` / `rover1234`).
2. Connect the phone to that network.
3. Open `http://192.168.4.1` — joystick left, gimbal sliders right.

## 6. Tests (no hardware needed)

```sh
rover/.venv/bin/python -m unittest discover -s rover/tests
rover/.venv/bin/python rover/gui/rover_gui.py --sim --selftest
```

## Editing protocol changes

`shared/protocol.h` is canonical. Edit it, then:

```sh
cp rover/shared/protocol.h rover/rover/protocol.h
cp rover/shared/protocol.h rover/base/protocol.h
cp rover/shared/protocol.h rover/esp32cam/protocol.h
```

and mirror any payload changes into `gui/protocol.py` (the parity test
will catch drift: `tests/test_protocol.py` compiles the C header with
`cc` and diffs its output against Python).
