#!/usr/bin/env python3
"""Rover GUI: keyboard drive + live camera + telemetry dashboard.

Usage:
  rover_gui.py --port /dev/cu.usbmodemXXXX     (real hardware, base Uno)
  rover_gui.py --sim                           (fake telemetry + fake video)
  rover_gui.py --sim --selftest                (headless-ish 3s smoke test)

Keys:
  W/S throttle   A/D steering   arrows gimbal   SPACE stop   ESC quit
"""
import argparse
import math
import os
import queue
import sys
import threading
import time

import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as proto

CMD_INTERVAL = 0.05      # 20 Hz command rate
PING_INTERVAL = 1.0
GIMBAL_STEP = 4          # deg per command while held
THRUST_RATE = 12         # speed units per frame toward key target
WIN_W, WIN_H = 1040, 640
VID_W, VID_H = 800, 480
BATT_FULL = 8.4
BATT_EMPTY = 6.0

BLACK = (10, 10, 12)
GRAY = (90, 95, 100)
WHITE = (235, 235, 235)
GREEN = (70, 200, 90)
RED = (220, 60, 60)
AMBER = (235, 180, 60)


# --------------------------------------------------------------------------
# links
# --------------------------------------------------------------------------
class BaseLink(threading.Thread):
    """Owns the protocol stream. Reader thread pushes frames to a queue."""

    def __init__(self):
        super().__init__(daemon=True)
        self.parser = proto.Parser()
        self.frames = queue.Queue(maxsize=64)
        self.telemetry = None
        self.rtt_ms = None
        self._ping_sent = None
        self._lock = threading.Lock()

    def run(self):
        while True:
            try:
                self._read_loop()
            except Exception as exc:  # serial drop etc.
                print(f"link error: {exc}", file=sys.stderr)
                time.sleep(1)

    def _read_loop(self):
        raise NotImplementedError

    def send(self, frame: bytes):
        raise NotImplementedError

    def ping(self):
        self.send(proto.make_ping())

    def handle_frame(self, ftype, payload):
        if ftype == proto.TELEM:
            with self._lock:
                self.telemetry = proto.parse_telem(payload)
        elif ftype == proto.ACK:
            with self._lock:
                if self._ping_sent:
                    self.rtt_ms = (time.monotonic() - self._ping_sent) * 1000
                    self._ping_sent = None


class SerialLink(BaseLink):
    def __init__(self, port, baud=115200):
        super().__init__()
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._tx = threading.Lock()
        self.start()

    def _read_loop(self):
        data = self.ser.read(128)
        for frame in self.parser.feed(data):
            self.handle_frame(*frame)

    def send(self, frame: bytes):
        with self._tx:
            self.ser.write(frame)

    def ping(self):
        with self._lock:
            self._ping_sent = time.monotonic()
        self.send(proto.make_ping())


class SimLink(BaseLink):
    def __init__(self):
        super().__init__()
        self.start()

    def _read_loop(self):
        time.sleep(0.02)
        t = time.monotonic()
        dist = int(50 + 40 * math.sin(t / 2) + abs(t % 7 - 3.5) * 30)
        batt = 7400 + int(300 * math.sin(t / 5))
        payload = bytes([
            dist >> 8, dist & 0xFF, batt >> 8, batt & 0xFF,
            int(3 + 6 * abs(math.sin(t / 9))), 1,
        ])
        frame = proto.encode(proto.TELEM, payload)
        for parsed in self.parser.feed(frame):
            self.handle_frame(*parsed)
        if self._ping_sent and time.monotonic() - self._ping_sent > 0.03:
            self.handle_frame(proto.ACK, b"")

    def send(self, frame: bytes):
        pass  # sim ignores outgoing frames

    def ping(self):
        with self._lock:
            self._ping_sent = time.monotonic()


# --------------------------------------------------------------------------
# video
# --------------------------------------------------------------------------
class VideoSource(threading.Thread):
    def __init__(self, url=None, sim=False):
        super().__init__(daemon=True)
        self.url = url
        self.sim = sim
        self.latest = None
        self.ready = threading.Event()
        self._t0 = time.monotonic()
        self.start()

    def run(self):
        if self.sim:
            self._run_sim()
        else:
            self._run_capture()

    def _run_sim(self):
        while True:
            t = time.monotonic() - self._t0
            frame = np.full((VID_H, VID_W, 3), 18, dtype=np.uint8)
            x = int(VID_W / 2 + 200 * math.sin(t / 2.1))
            y = int(VID_H / 2 + 120 * math.sin(t / 1.4))
            frame[max(0, y - 20):y + 20, max(0, x - 30):x + 30] = (40, 190, 90)
            frame[:60, :200] = (70, 70, 90)
            self.latest = frame
            self.ready.set()
            time.sleep(0.05)

    def _run_capture(self):
        import cv2
        cap = cv2.VideoCapture(self.url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while True:
            if not cap.isOpened():
                time.sleep(1.0)
                cap = cv2.VideoCapture(self.url)
                continue
            ok, frame = cap.read()
            if ok:
                frame = cv2.resize(frame, (VID_W, VID_H))
                self.latest = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.ready.set()
            else:
                time.sleep(0.5)


# --------------------------------------------------------------------------
# drawing helpers
# --------------------------------------------------------------------------
def draw_bar(surf, x, y, w, h, frac, color, label, text):
    pygame.draw.rect(surf, (40, 42, 48), (x, y, w, h))
    fill = int(frac * (w - 2))
    if fill > 0:
        pygame.draw.rect(surf, color, (x + 1, y + 1, fill, h - 2))
    surf.blit(label, (x, y - 18))
    surf.blit(text, (x + w + 8, y - 2))


def main():
    ap = argparse.ArgumentParser(description="Rover control GUI")
    ap.add_argument("--port", default=None, help="serial port of base Uno")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim", action="store_true", help="fake link + fake video")
    ap.add_argument("--camera", default="http://192.168.4.1:81/stream",
                    help="MJPEG stream URL (use --camera none to disable)")
    ap.add_argument("--selftest", action="store_true",
                    help="run 3s in sim mode, exit 0 on success")
    args = ap.parse_args()

    if args.selftest:
        args.sim = True
        args.camera = "none"

    link = SimLink() if args.sim else SerialLink(args.port, args.baud)
    video = VideoSource(None if args.camera == "none" else args.camera,
                        sim=args.sim)

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Rover")
    font = pygame.font.SysFont("menlo", 17)
    small = pygame.font.SysFont("menlo", 14)
    clock = pygame.time.Clock()

    throttle = steer = 0
    pan = tilt = 90
    last_cmd = 0.0
    last_ping = 0.0
    last_cam_ok = False
    running = True
    selftest_start = time.monotonic() if args.selftest else None

    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    throttle = steer = 0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            throttle = min(255, throttle + THRUST_RATE * 4)
        if keys[pygame.K_s]:
            throttle = max(-255, throttle - THRUST_RATE * 4)
        if keys[pygame.K_a]:
            steer = max(-255, steer - THRUST_RATE * 4)
        if keys[pygame.K_d]:
            steer = min(255, steer + THRUST_RATE * 4)
        if not (keys[pygame.K_w] or keys[pygame.K_s]):
            throttle = 0
        if not (keys[pygame.K_a] or keys[pygame.K_d]):
            steer = 0
        if keys[pygame.K_LEFT]:
            pan = max(0, pan - GIMBAL_STEP)
        if keys[pygame.K_RIGHT]:
            pan = min(180, pan + GIMBAL_STEP)
        if keys[pygame.K_UP]:
            tilt = max(0, tilt - GIMBAL_STEP)
        if keys[pygame.K_DOWN]:
            tilt = min(180, tilt + GIMBAL_STEP)

        now = time.monotonic()
        if now - last_cmd >= CMD_INTERVAL and (throttle or steer):
            link.send(proto.make_drive(throttle, steer))
            last_cmd = now
        if now - last_cmd >= CMD_INTERVAL and keys[pygame.K_SPACE]:
            link.send(proto.make_drive(0, 0))
            last_cmd = now
        if keys[pygame.K_LEFT] or keys[pygame.K_RIGHT] or \
           keys[pygame.K_UP] or keys[pygame.K_DOWN]:
            link.send(proto.make_gimbal(pan, tilt))
        if now - last_ping >= PING_INTERVAL:
            link.ping()
            last_ping = now

        # ---- draw ----
        screen.fill(BLACK)

        vid_surf = None
        if video.ready.is_set():
            frame = video.latest
            try:
                vid_surf = pygame.image.frombuffer(frame.tobytes(),
                                                   (VID_W, VID_H), "RGB")
                last_cam_ok = True
            except Exception:
                last_cam_ok = False
        if vid_surf:
            screen.blit(vid_surf, (0, 0))
        else:
            pygame.draw.rect(screen, (28, 30, 36), (0, 0, VID_W, VID_H))
            msg = "SIM VIDEO" if args.sim else "NO VIDEO - check ESP32-CAM stream"
            screen.blit(font.render(msg, True, GRAY),
                        (VID_W // 2 - 140, VID_H // 2))
        if not args.sim and not last_cam_ok and not video.ready.is_set():
            screen.blit(small.render("connecting to camera...", True, AMBER),
                        (10, 10))

        tele = None
        with link._lock:
            tele = link.telemetry
            rtt = link.rtt_ms
            ping_sent = link._ping_sent

        x = VID_W + 24
        y = 24
        info = [
            ("THROTTLE", throttle),
            ("STEER", steer),
            ("PAN", pan),
            ("TILT", tilt),
        ]
        for name, val in info:
            t = font.render(f"{name}: {val:+d}", True, WHITE)
            screen.blit(t, (x, y))
            y += 26

        if tele:
            y += 8
            dist = tele["dist_cm"]
            batt = tele["batt_mv"] / 1000.0
            label = small.render("DIST (cm)", True, GRAY)
            text = font.render(f"{dist:>4}", True, WHITE)
            draw_bar(screen, x, y + 18, 180, 14,
                     min(1.0, dist / 400.0), GREEN, label, text)
            y += 50
            label = small.render("BATTERY (V)", True, GRAY)
            text = font.render(f"{batt:3.2f}", True, WHITE)
            draw_bar(screen, x, y + 18, 180, 14,
                     (batt - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY),
                     GREEN if batt > 7.0 else RED, label, text)
            y += 50
            label = small.render("ROVER LOOP (ms)", True, GRAY)
            text = font.render(f"{tele['loop_ms']:>4}", True, WHITE)
            draw_bar(screen, x, y + 18, 180, 14,
                     min(1.0, tele["loop_ms"] / 30.0), AMBER, label, text)
            y += 50
            if tele["radio_ok"]:
                screen.blit(font.render("RADIO: OK", True, GREEN), (x, y))
            else:
                screen.blit(font.render("RADIO: NONE", True, RED), (x, y))
            y += 30
            if rtt is not None:
                screen.blit(font.render(f"LATENCY: {rtt:5.1f} ms", True, WHITE),
                            (x, y))
                y += 30
            if ping_sent:
                screen.blit(font.render("PING...", True, AMBER), (x, y))
                y += 30

        y += 10
        ok, bad = link.parser.frames_ok, link.parser.frames_bad
        screen.blit(font.render(f"frames ok/bad: {ok}/{bad}", True, GRAY), (x, y))
        y += 24
        screen.blit(small.render("WASD drive  arrows gimbal  SPACE stop  ESC quit",
                                 True, GRAY), (x, y))

        pygame.display.flip()

        if selftest_start and time.monotonic() - selftest_start > 3.0:
            if link.parser.frames_ok > 5:
                print(f"SELFTEST PASS: {link.parser.frames_ok} frames received")
                running = False
            else:
                print(f"SELFTEST FAIL: only {link.parser.frames_ok} frames")
                running = False

    pygame.quit()
    if selftest_start:
        sys.exit(0 if link.parser.frames_ok > 5 else 1)


if __name__ == "__main__":
    main()
