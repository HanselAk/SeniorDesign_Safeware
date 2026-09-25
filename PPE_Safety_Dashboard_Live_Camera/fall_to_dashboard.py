"""Read ESP32 MPU6050 acceleration over USB; detect and log fall candidates.

Run from the Jetson inside ppe_env while demo_dashboard.py is running.
Use --demo first to test the detector and dashboard without a sensor.
"""

import argparse
import csv
import json
import math
import os
import time
import urllib.error
import urllib.request
from datetime import datetime


URL = "http://127.0.0.1:5000/api/fall/update"
FIELDS = ("timestamp", "ax", "ay", "az", "smv", "event")
FREE_FALL_THRESHOLD = 3.0  # m/s^2
IMPACT_THRESHOLD = 20.0     # m/s^2
IMPACT_WINDOW = 1.5         # seconds


class FallDetector:
    """Require two low samples, then a timely impact; ignore delayed impacts."""

    def __init__(self):
        self.low_time = None
        self.freefall_time = None
        self.cooldown_until = 0.0

    def reset(self):
        self.low_time = None
        self.freefall_time = None

    def process(self, smv, now):
        if now < self.cooldown_until:
            return ""
        if self.freefall_time is not None:
            if now - self.freefall_time > IMPACT_WINDOW:
                self.reset()
                return "FREE_FALL_EXPIRED"
            if smv >= IMPACT_THRESHOLD:
                self.reset()
                self.cooldown_until = now + 3.0
                return "FALL_DETECTED"
            return ""
        if smv < FREE_FALL_THRESHOLD:
            if self.low_time is not None and now - self.low_time <= 0.1:
                self.freefall_time = now
                self.low_time = None
                return "FREE_FALL"
            self.low_time = now
        else:
            self.low_time = None
        return ""


def send_sample(row):
    body = {key: row[key] for key in ("ax", "ay", "az", "event")}
    req = urllib.request.Request(URL, json.dumps(body).encode("utf-8"),
                                 {"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=2):
            pass
    except (urllib.error.URLError, TimeoutError) as exc:
        raise RuntimeError("Dashboard is unavailable. Start demo_dashboard.py first.") from exc


def samples_from_serial(port):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial in ppe_env: python -m pip install pyserial") from exc
    print(f"Opening {port} at 115200 baud. Close Arduino Serial Monitor first.", flush=True)
    with serial.Serial(port, 115200, timeout=1) as connection:
        time.sleep(2)  # Most ESP32 boards reset when the USB serial port opens.
        last_sample = time.monotonic()
        while True:
            line = connection.readline().decode("utf-8", errors="replace").strip()
            if not line:
                if time.monotonic() - last_sample > 3:
                    raise RuntimeError("No MPU6050 samples for 3 seconds. Check ESP32 wiring and serial port.")
                continue
            if line.startswith("#"):
                print(line)
                continue
            try:
                sample = json.loads(line)
                axes = tuple(float(sample[key]) for key in ("ax", "ay", "az"))
                if not all(math.isfinite(a) and abs(a) <= 160 for a in axes):
                    continue
            except (ValueError, KeyError, TypeError):
                continue
            last_sample = time.monotonic()
            yield axes


def detect_port():
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("Install pyserial in ppe_env: python -m pip install pyserial") from exc
    ports = [p.device for p in list_ports.comports()
             if p.device.startswith(("/dev/ttyUSB", "/dev/ttyACM"))]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one ESP32 USB port; found {ports}. Run with --port /dev/ttyUSB0 or /dev/ttyACM0 as appropriate.")
    return ports[0]


def demo_samples():
    print("Demo: rest, two low readings, then impact. This will create a TEST fall alert.")
    for axes in [(0, 0, 9.81)] * 12 + [(0.1, 0.1, 1.0)] * 3 + [(0, 0, 25.0)] + [(0, 0, 9.81)] * 12:
        yield axes
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 USB serial port, e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    parser.add_argument("--demo", action="store_true", help="Send one test fall without ESP32 hardware")
    args = parser.parse_args()
    source = demo_samples() if args.demo else samples_from_serial(args.port or detect_port())
    log_path = "fall_log.csv"
    detector = FallDetector()
    count = 0
    needs_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
    with open(log_path, "a", newline="", encoding="utf-8") as logfile:
        writer = csv.DictWriter(logfile, fieldnames=FIELDS)
        if needs_header:
            writer.writeheader()
        print(f"Fall detector running. Log: {log_path}. Ctrl+C to stop.", flush=True)
        try:
            for ax, ay, az in source:
                now = time.monotonic()
                smv = math.sqrt(ax * ax + ay * ay + az * az)
                event = detector.process(smv, now)
                row = {"timestamp": datetime.now().isoformat(timespec="milliseconds"),
                       "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                       "smv": round(smv, 4), "event": event}
                count += 1
                if event or count % 10 == 0:
                    writer.writerow(row)
                    logfile.flush()
                if event or count % 5 == 0:
                    send_sample(row)
                if event:
                    print(f"{event}: SMV={smv:.2f} m/s^2", flush=True)
        except KeyboardInterrupt:
            print("\nStopped fall detector.")
    if args.demo:
        print("Demo complete. Check the dashboard fall alert, then acknowledge it.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
