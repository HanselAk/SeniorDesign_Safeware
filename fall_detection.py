"""
MPU6050 Fall Detection - Raspberry Pi
Library: mpu6050-raspberrypi (pip install mpu6050-raspberrypi)
Algorithm: Threshold-based SMV (Signal Magnitude Vector)


MPU6050Raspberry PiVCCPin 1 (3.3V)GNDPin 6 (GND)SDAPin 3 (GPIO 2)SCLPin 5 (GPIO 3)AD0GND


Fall logic:
  1. SMV drops below FREE_FALL_THRESHOLD  -> free-fall phase detected
  2. Within IMPACT_WINDOW seconds, SMV spikes above IMPACT_THRESHOLD -> impact confirmed
  3. Event is logged with timestamp and raw sensor values
"""

import math
import time
import csv
import os
from mpu6050 import mpu6050

# ── Sensor setup ────────────────────────────────────────────────────────────
SENSOR_ADDRESS = 0x68          # change to 0x69 if AD0 is HIGH
sensor = mpu6050(SENSOR_ADDRESS)

# ── Thresholds ───────────────────────────────────────────────────────────────
# SMV at rest (1g gravity) ≈ 9.8 m/s²
# Free-fall: body acceleration near 0g  → SMV drops below ~3 m/s²
# Impact:   sudden deceleration         → SMV spikes above ~20 m/s²
# Tune these for your use case / sensor placement
FREE_FALL_THRESHOLD = 3.0      # m/s²  — SMV below this triggers free-fall watch
IMPACT_THRESHOLD    = 20.0     # m/s²  — SMV above this after free-fall = fall
IMPACT_WINDOW       = 1.5      # seconds to wait for impact after free-fall
SAMPLE_RATE         = 0.02     # seconds between samples (50 Hz)

# ── Log file setup ───────────────────────────────────────────────────────────
LOG_FILE = "fall_log.csv"
FIELDS   = ["timestamp", "ax", "ay", "az", "smv", "event"]

def init_log():
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=FIELDS).writeheader()

def log_row(ax, ay, az, smv, event=""):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    row = {
        "timestamp": ts,
        "ax": round(ax, 4),
        "ay": round(ay, 4),
        "az": round(az, 4),
        "smv": round(smv, 4),
        "event": event
    }
    with open(LOG_FILE, "a", newline="") as f:
        csv.DictWriter(f, fieldnames=FIELDS).writerow(row)
    return row

# ── SMV calculation ───────────────────────────────────────────────────────────
def get_smv(accel):
    return math.sqrt(accel["x"]**2 + accel["y"]**2 + accel["z"]**2)

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    init_log()
    print(f"[INFO] Fall detection started. Logging to: {LOG_FILE}")
    print(f"[INFO] Free-fall threshold: {FREE_FALL_THRESHOLD} m/s²")
    print(f"[INFO] Impact threshold:    {IMPACT_THRESHOLD} m/s²")
    print(f"[INFO] Impact window:       {IMPACT_WINDOW}s")
    print("─" * 50)

    in_freefall = False
    freefall_time = None

    try:
        while True:
            accel = sensor.get_accel_data()
            smv   = get_smv(accel)
            ax, ay, az = accel["x"], accel["y"], accel["z"]

            event = ""

            # ── Phase 1: Detect free-fall ────────────────────────────────────
            if not in_freefall and smv < FREE_FALL_THRESHOLD:
                in_freefall   = True
                freefall_time = time.time()
                event         = "FREE_FALL"
                print(f"[ALERT] Free-fall detected! SMV={smv:.2f}")
                log_row(ax, ay, az, smv, event)

            # ── Phase 2: Detect impact within window ─────────────────────────
            elif in_freefall:
                elapsed = time.time() - freefall_time

                if smv > IMPACT_THRESHOLD:
                    event = "FALL_DETECTED"
                    print(f"[FALL ] FALL DETECTED at {time.strftime('%H:%M:%S')} | SMV={smv:.2f}")
                    log_row(ax, ay, az, smv, event)
                    in_freefall = False

                elif elapsed > IMPACT_WINDOW:
                    # Free-fall phase expired without impact — false alarm
                    in_freefall = False
                    print(f"[INFO] Free-fall expired without impact (possible crouch)")

            # ── Normal sample logging (every 10th sample = 5Hz to keep file lean) ──
            else:
                log_row(ax, ay, az, smv)

            # ── Console display ──────────────────────────────────────────────
            print(
                f"ax={ax:7.3f}  ay={ay:7.3f}  az={az:7.3f}  "
                f"SMV={smv:7.3f}  {'<<' + event + '>>' if event else ''}",
                end="\r"
            )

            time.sleep(SAMPLE_RATE)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

if __name__ == "__main__":
    main()
