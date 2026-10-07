"""Dashboard with real YOLO camera results, ESP32 wearable motion/PDR and simulated vitals."""

from __future__ import annotations

import math
import os
import base64
import binascii
import json
import random
import threading
import time
from collections import deque
from datetime import datetime

from flask import Flask, Response, jsonify, render_template, request


app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 2 * 1024 * 1024
state_lock = threading.Lock()
stop_event = threading.Event()

BASE_LAT = 33.952600
BASE_LON = -84.549900
MAX_LOG_ITEMS = 20
HISTORY_POINTS = 60

state = {
    "ax": 0.0,
    "ay": 0.0,
    "az": 9.81,
    "smv": 9.81,
    "lat": BASE_LAT,
    "lon": BASE_LON,
    "alt": 310.0,
    "helmet_status": "UNKNOWN",
    "vest_status": "UNKNOWN",
    "temp_f": 98.2,
    "temp_c": 36.8,
    "hr_bpm": 78,
    "fall_active": False,
    "motion_has_connected": False,
    "motion_last_update": 0.0,
    # Pedestrian dead reckoning from the ESP32 MPU9250, meters from the reset point.
    "pdr": {"x": 0.0, "y": 0.0, "heading": 0.0, "distance": 0.0, "steps": 0, "walking": False},
    "temp_alert": False,
    "hr_alert": False,
    "detections": [],
    "fps": 0.0,
    "camera_last_update": 0.0,
    "camera_frame": None,
    "workers": [],
    "alerts": [],
    "last_update": time.time(),
}

temp_history: deque[float] = deque([98.2] * HISTORY_POINTS, maxlen=HISTORY_POINTS)
hr_history: deque[int] = deque([78] * HISTORY_POINTS, maxlen=HISTORY_POINTS)

# Set SAFEWARE_TOKEN in the environment to require X-Safeware-Token on ingest routes.
API_TOKEN = os.environ.get("SAFEWARE_TOKEN", "")


def authorized() -> bool:
    """Localhost (the camera and USB bridge scripts) is always allowed; remote devices need the token."""
    if not API_TOKEN or request.remote_addr in ("127.0.0.1", "::1"):
        return True
    return request.headers.get("X-Safeware-Token", "") == API_TOKEN


def overall_status() -> str:
    if state["fall_active"] or state["temp_alert"] or state["hr_alert"]:
        return "EMERGENCY"
    if state["motion_has_connected"] and time.time() - state["motion_last_update"] >= 3:
        return "UNVERIFIED"
    if camera_online() and (state["helmet_status"] == "MISSING" or state["vest_status"] == "MISSING"):
        return "WARNING"
    if camera_online() and state["helmet_status"] == state["vest_status"] == "DETECTED":
        return "SAFE"
    return "UNVERIFIED"


def camera_online() -> bool:
    return time.time() - state["camera_last_update"] < 4


def ppe_status(scores: dict[str, float], positive: tuple[str, ...], negative: str) -> str:
    good = max((scores.get(label, 0.0) for label in positive), default=0.0)
    bad = scores.get(negative, 0.0)
    if bad >= 0.5 and bad > good:
        return "MISSING"
    if good >= 0.5 and good > bad:
        return "DETECTED"
    return "UNKNOWN"


def add_alert(kind: str, message: str, severity: str = "critical") -> None:
    state["alerts"].insert(
        0,
        {
            "id": f"{time.time_ns()}",
            "type": kind,
            "message": message,
            "severity": severity,
            "time": datetime.now().strftime("%I:%M:%S %p"),
            "lat": state["lat"],
            "lon": state["lon"],
        },
    )
    del state["alerts"][MAX_LOG_ITEMS:]


# ---------------------------------------------------------------------------
# Wearable motion engine. The ESP32 (esp32_safeware.ino) streams raw MPU9250
# samples to /api/imu; this turns them into heading, steps, position (PDR) and
# fall events. Same logic that used to run on the ESP32 in esp32_motion.ino.
# Accel is in g, gyro in deg/s, magnetometer in uT, times in sensor seconds.
# ---------------------------------------------------------------------------
SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pdr_settings.json")

STEP_HI = 0.12              # g, vertical accel peak that counts as a step
STEP_LO = -0.05             # g, must dip below this before the next step
STEP_MIN_S = 0.28
WALK_TIMEOUT_S = 1.5
K_MAG = 0.5                 # magnetometer drift correction speed (per second)
B_TOL = 0.20                # ignore mag if field strength changes more than 20%
DIP_TOL = 10.0              # ignore mag if dip angle changes more than 10 degrees
MAX_TURN_RATE = 90.0        # skip mag correction during fast turns (deg/s)
MAGCAL_S = 30.0
GYRO_CAL_SAMPLES = 200      # 2 s at 100 Hz, sensor must be still
FREE_FALL_G = 0.50          # two samples below this within 100 ms = free fall
IMPACT_G = 1.8              # impact that confirms the fall
IMPACT_WINDOW_S = 1.5
FALL_COOLDOWN_S = 3.0
MAX_PATH_POINTS = 1500


def norm3(v) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def cross3(a, b) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def wrap360(a: float) -> float:
    return a % 360.0


def wrap180(a: float) -> float:
    a = wrap360(a)
    return a - 360.0 if a > 180.0 else a


class MotionEngine:
    def __init__(self) -> None:
        # Tuning and magnetometer calibration, saved to pdr_settings.json.
        self.step_length = 0.70
        self.snap = 0                     # 0 = off, 45 or 90 degrees
        self.fuse = True
        self.mag_offset = [-100.35, -255.51, -377.74]
        self.mag_scale = [1.367, 0.992, 0.793]
        self.load_settings()

        self.device = ""
        self.boot = None
        self.imu_ok = self.mag_ok = False
        self.dropped = 0
        self.last_packet = 0.0
        self.last_us: int | None = None
        self.clock = 0.0
        self.last_accel = [0.0, 0.0, 1.0]
        self.mag_msg = ""
        self.session = 0
        self.path: deque[tuple[float, float]] = deque(maxlen=MAX_PATH_POINTS)
        self.path_count = 0
        self.restart()

    # ---- settings ----
    def load_settings(self) -> None:
        try:
            with open(SETTINGS_PATH) as f:
                saved = json.load(f)
            self.step_length = min(1.2, max(0.3, float(saved.get("step_length", self.step_length))))
            self.snap = int(saved.get("snap", self.snap)) if saved.get("snap") in (0, 45, 90) else 0
            self.fuse = bool(saved.get("fuse", self.fuse))
            if len(saved.get("mag_offset", [])) == 3 and len(saved.get("mag_scale", [])) == 3:
                self.mag_offset = [float(v) for v in saved["mag_offset"]]
                self.mag_scale = [float(v) for v in saved["mag_scale"]]
                print("Loaded magnetometer calibration from pdr_settings.json")
        except (OSError, ValueError, TypeError, AttributeError):
            pass

    def save_settings(self) -> None:
        try:
            with open(SETTINGS_PATH, "w") as f:
                json.dump({"step_length": self.step_length, "snap": self.snap, "fuse": self.fuse,
                           "mag_offset": self.mag_offset, "mag_scale": self.mag_scale}, f, indent=2)
        except OSError as exc:
            print(f"Could not save {SETTINGS_PATH}: {exc}")

    # ---- control ----
    def restart(self) -> None:
        """Fresh ESP32 boot: recalibrate the gyro and start a new path."""
        self.g_bias = [0.0, 0.0, 0.0]
        self.grav = [0.0, 0.0, 1.0]
        self.ref_axis = 0
        self.mag_v: list[float] | None = None   # latest calibrated mag vector, consumed by update_pdr
        self.mag_state = 0                       # 0 no data, 1 good, 2 disturbed
        self.b_now = self.mag_heading = 0.0
        self.mag_cal_start: float | None = None
        self.cmn = [math.inf] * 3
        self.cmx = [-math.inf] * 3
        self.in_free_fall = self.low_seen = self.had_fall = False
        self.low_t = self.free_fall_t = self.last_fall_t = 0.0
        self.start_gyro_cal()
        self.reset_pdr()

    def reset_pdr(self) -> None:
        self.x = self.y = self.heading = self.distance = 0.0
        self.steps = 0
        self.av_filt = 0.0
        self.armed = True
        self.last_step_t = -math.inf
        self.walking = False
        self.still_s = 0.0
        self.path.clear()
        self.path_count = 0
        self.add_point(0.0, 0.0)
        self.ref_set = False                     # re-anchor magnetometer to the new facing direction
        self.mag_ref = self.b_ref = self.dip_ref = 0.0
        self.session += 1

    def start_gyro_cal(self) -> None:
        self.calibrating = True
        self.cal_count = 0
        self.cal_g = [0.0, 0.0, 0.0]
        self.cal_a = [0.0, 0.0, 0.0]

    def start_mag_cal(self) -> bool:
        if not self.mag_ok:
            return False
        self.cmn = [math.inf] * 3
        self.cmx = [-math.inf] * 3
        self.mag_cal_start = self.clock
        self.mag_msg = "Rotate the sensor in every direction"
        return True

    def configure(self, length=None, snap=None, fuse=None) -> None:
        if length is not None:
            self.step_length = min(1.2, max(0.3, float(length)))
        if snap is not None:
            self.snap = int(snap) if int(snap) in (45, 90) else 0
        if fuse is not None:
            self.fuse = bool(fuse)
        self.save_settings()

    # ---- processing ----
    def ingest(self, device: str, boot: int, imu_ok: bool, mag_ok: bool, dropped: int,
               samples: list[list[float]]) -> list[tuple[float, int]]:
        """Process one batch from the ESP32. Returns (peak_g, severity) for each new fall."""
        if boot != self.boot:
            print(f"Wearable {device or '?'} connected (boot {boot}): hold still, calibrating gyro")
            self.boot = boot
            self.last_us = None
            self.restart()
        self.device, self.imu_ok, self.mag_ok, self.dropped = device, imu_ok, mag_ok, dropped
        self.last_packet = time.time()
        falls = []
        for row in samples:
            us = int(row[0])
            if self.last_us is None:
                dt = 0.01
            else:
                gap = (us - self.last_us) % 2**32      # micros() wraps every ~71 minutes
                if gap == 0 or gap >= 2**31:
                    continue                           # already processed (resent batch)
                dt = min(gap / 1e6, 0.1)
            self.last_us = us
            self.clock += dt
            fall = self.process_sample(row[1:4], row[4:7], row[7:10] if len(row) == 10 else None, dt)
            if fall:
                falls.append(fall)
        return falls

    def process_sample(self, a, g, mag, dt):
        self.last_accel = list(a)
        fall = self.detect_fall(a)
        if mag is not None and self.mag_ok:
            self.process_mag(mag)
        if self.mag_cal_start is not None and self.clock - self.mag_cal_start >= MAGCAL_S:
            self.finish_mag_cal()

        if self.calibrating:
            for i in range(3):
                self.cal_g[i] += g[i]
                self.cal_a[i] += a[i]
            self.cal_count += 1
            if self.cal_count >= GYRO_CAL_SAMPLES:
                self.g_bias = [v / GYRO_CAL_SAMPLES for v in self.cal_g]
                self.grav = [v / GYRO_CAL_SAMPLES for v in self.cal_a]
                self.ref_axis = min(range(3), key=lambda i: abs(self.grav[i]))
                self.ref_set = False
                self.calibrating = False
                print("Gyro bias: {:.2f} {:.2f} {:.2f} dps".format(*self.g_bias))
            return fall

        self.update_pdr(a, list(g), dt)
        return fall

    def process_mag(self, ut) -> None:
        if self.mag_cal_start is not None:
            for i in range(3):
                self.cmn[i] = min(self.cmn[i], ut[i])
                self.cmx[i] = max(self.cmx[i], ut[i])
            return
        c = [(ut[i] - self.mag_offset[i]) * self.mag_scale[i] for i in range(3)]
        self.mag_v = [c[1], c[0], -c[2]]          # align magnetometer axes with accel/gyro axes

    def finish_mag_cal(self) -> None:
        self.mag_cal_start = None
        r = [(self.cmx[i] - self.cmn[i]) / 2.0 for i in range(3)]
        if min(r) < 20:
            self.mag_msg = "Not enough rotation. Kept old calibration."
            return
        avg = sum(r) / 3.0
        self.mag_offset = [round((self.cmx[i] + self.cmn[i]) / 2.0, 2) for i in range(3)]
        self.mag_scale = [round(avg / r[i], 3) for i in range(3)]
        self.save_settings()
        self.ref_set = False
        self.mag_msg = f"Saved. Field strength {avg:.0f} uT"
        print(f"Mag calibration saved. Offset {self.mag_offset}  Scale {self.mag_scale}")

    def update_pdr(self, a, gyro, dt) -> None:
        now = self.clock
        for i in range(3):
            gyro[i] -= self.g_bias[i]

        # Gravity estimate gives "up" in the sensor frame.
        for i in range(3):
            self.grav[i] += 0.02 * (a[i] - self.grav[i])
        gn = norm3(self.grav)
        if gn < 0.5:
            return
        u = [v / gn for v in self.grav]

        # Gyro heading.
        yaw_rate = gyro[0] * u[0] + gyro[1] * u[1] + gyro[2] * u[2]
        if abs(yaw_rate) < 0.3:
            yaw_rate = 0.0
        self.heading = wrap360(self.heading - yaw_rate * dt)

        # Magnetometer correction.
        if self.mag_ok and self.mag_v is not None and self.mag_cal_start is None:
            m, self.mag_v = self.mag_v, None
            mn = norm3(m)
            self.b_now = mn
            if mn > 5:
                d = [-u[0], -u[1], -u[2]]                      # down
                east = cross3(d, m)
                en = norm3(east)
                if en > 1e-3:
                    east = [v / en for v in east]
                    north = cross3(east, d)
                    mh = math.degrees(math.atan2(east[self.ref_axis], north[self.ref_axis]))
                    dip = math.degrees(math.asin(max(-1.0, min(1.0, (m[0] * d[0] + m[1] * d[1] + m[2] * d[2]) / mn))))
                    if not self.ref_set:
                        self.mag_ref = wrap360(mh - self.heading)
                        self.b_ref, self.dip_ref = mn, dip
                        self.ref_set = True
                    self.mag_heading = wrap360(mh - self.mag_ref)
                    good = abs(mn - self.b_ref) < B_TOL * self.b_ref and abs(dip - self.dip_ref) < DIP_TOL
                    self.mag_state = 1 if good else 2
                    if good and self.fuse and abs(yaw_rate) < MAX_TURN_RATE:
                        self.heading = wrap360(self.heading + K_MAG * wrap180(self.mag_heading - self.heading) * dt)

        # Step detection.
        av = a[0] * u[0] + a[1] * u[1] + a[2] * u[2] - gn
        self.av_filt += 0.3 * (av - self.av_filt)
        if self.armed and self.av_filt > STEP_HI and now - self.last_step_t > STEP_MIN_S:
            self.armed = False
            self.last_step_t = now
            self.steps += 1
            h = round(self.heading / self.snap) * self.snap if self.snap else self.heading
            self.x += self.step_length * math.sin(math.radians(h))
            self.y += self.step_length * math.cos(math.radians(h))
            self.distance += self.step_length
            self.add_point(self.x, self.y)
        if not self.armed and self.av_filt < STEP_LO:
            self.armed = True
        self.walking = self.steps > 0 and now - self.last_step_t < WALK_TIMEOUT_S

        # Slow gyro bias correction while still.
        if abs(norm3(a) - 1.0) < 0.03 and norm3(gyro) < 2.0 and not self.walking:
            self.still_s += dt
        else:
            self.still_s = 0.0
        if self.still_s > 1.0:
            for i in range(3):
                self.g_bias[i] += 0.002 * gyro[i]

    def detect_fall(self, a) -> tuple[float, int] | None:
        """Free fall (two low samples within 100 ms), then an impact within 1.5 s."""
        now = self.clock
        if self.had_fall and now - self.last_fall_t < FALL_COOLDOWN_S:
            return None
        an = norm3(a)
        if self.in_free_fall:
            if now - self.free_fall_t > IMPACT_WINDOW_S:
                self.in_free_fall = False
            elif an >= IMPACT_G:
                self.in_free_fall = False
                self.had_fall = True
                self.last_fall_t = now
                severity = 3 if an >= 3.5 else 2 if an >= 2.75 else 1
                print(f"FALL DETECTED: impact {an:.2f} g, severity {severity}")
                return round(an, 2), severity
            return None
        if an < FREE_FALL_G:
            if self.low_seen and now - self.low_t <= 0.1:
                self.in_free_fall = True
                self.free_fall_t = now
                self.low_seen = False
            else:
                self.low_seen = True
                self.low_t = now
        else:
            self.low_seen = False
        return None

    # ---- output ----
    def add_point(self, x: float, y: float) -> None:
        self.path.append((round(x, 2), round(y, 2)))
        self.path_count += 1

    def online(self) -> bool:
        return time.time() - self.last_packet < 3

    def pdr_summary(self) -> dict:
        return {"x": round(self.x, 2), "y": round(self.y, 2), "heading": round(self.heading, 1),
                "distance": round(self.distance, 2), "steps": self.steps, "walking": self.walking}

    def snapshot(self, start: int, session: int) -> dict:
        """Live state plus path points from index `start`, for the /pdr map page."""
        if session != self.session:
            start = 0
        oldest = self.path_count - len(self.path)
        start = min(max(start, oldest), self.path_count)
        end = min(self.path_count, start + 300)
        points = list(self.path)[start - oldest:end - oldest]
        mag_left, spans = -1, [0, 0, 0]
        if self.mag_cal_start is not None:
            mag_left = max(0, int(MAGCAL_S - (self.clock - self.mag_cal_start)))
            spans = [round(self.cmx[i] - self.cmn[i]) if self.cmx[i] > self.cmn[i] else 0 for i in range(3)]
        return {
            "s": self.session, "next": end, "x": round(self.x, 2), "y": round(self.y, 2),
            "h": round(self.heading, 1), "steps": self.steps, "dist": round(self.distance, 2),
            "len": self.step_length, "snap": self.snap, "walk": self.walking,
            "cal": self.calibrating, "imu": self.imu_ok, "online": self.online(), "device": self.device,
            "dropped": self.dropped, "magok": self.mag_ok, "mag": self.mag_state, "fuse": self.fuse,
            "b": round(self.b_now, 1), "mh": round(self.mag_heading, 1), "mcal": mag_left,
            "msp": spans, "mmsg": self.mag_msg, "pts": points,
        }


motion_lock = threading.Lock()
motion = MotionEngine()


def simulator() -> None:
    phase = 0.0
    history_tick = 0
    while not stop_event.is_set():
        phase += 0.12
        history_tick += 1
        with state_lock:
            # Keep normal values moving gently unless a test alert is active.
            if not state["temp_alert"]:
                temp_f = 98.1 + math.sin(phase / 3.0) * 0.5 + random.uniform(-0.12, 0.12)
                state["temp_f"] = round(temp_f, 1)
                state["temp_c"] = round((temp_f - 32.0) * 5.0 / 9.0, 1)
            if not state["hr_alert"]:
                state["hr_bpm"] = max(62, min(108, int(79 + math.sin(phase) * 5 + random.uniform(-2, 2))))

            if not state["motion_has_connected"]:
                state["ax"] = round(math.sin(phase * 1.2) * 0.42 + random.uniform(-0.08, 0.08), 3)
                state["ay"] = round(math.cos(phase) * 0.35 + random.uniform(-0.08, 0.08), 3)
                state["az"] = round(9.81 + math.sin(phase * 0.7) * 0.2, 3)
                state["smv"] = round(
                    math.sqrt(state["ax"] ** 2 + state["ay"] ** 2 + state["az"] ** 2), 3
                )
            state["lat"] = round(BASE_LAT + math.sin(phase / 18.0) * 0.00024, 6)
            state["lon"] = round(BASE_LON + math.cos(phase / 20.0) * 0.00027, 6)
            state["alt"] = round(310 + math.sin(phase / 8.0) * 1.4, 1)
            state["last_update"] = time.time()

            if history_tick >= 4:
                history_tick = 0
                temp_history.append(state["temp_f"])
                hr_history.append(state["hr_bpm"])
        time.sleep(0.5)


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/api/state")
def api_state():
    with state_lock:
        snapshot = {key: value for key, value in state.items() if key != "camera_frame"}
        snapshot["pdr"] = dict(state["pdr"])
        snapshot["camera_online"] = camera_online()
        snapshot["motion_online"] = (state["motion_has_connected"] and
                                     time.time() - state["motion_last_update"] < 3)
        if not snapshot["camera_online"]:
            snapshot["helmet_status"] = "UNKNOWN"
            snapshot["vest_status"] = "UNKNOWN"
            snapshot["detections"] = []
            snapshot["fps"] = 0.0
            snapshot["workers"] = []
        snapshot["alerts"] = list(state["alerts"])
        snapshot["temp_history"] = list(temp_history)
        snapshot["hr_history"] = list(hr_history)
        snapshot["overall_status"] = overall_status()
        snapshot["connected"] = time.time() - state["last_update"] < 3
        return jsonify(snapshot)


@app.route("/api/camera/frame")
def camera_frame():
    with state_lock:
        frame = state["camera_frame"] if camera_online() else None
    if frame is None:
        return Response(status=404)
    return Response(frame, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/camera/update", methods=["POST"])
def camera_update():
    if not authorized():
        return jsonify({"error": "Unauthorized"}), 401
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict) or not isinstance(payload.get("detections"), list):
        return jsonify({"error": "Expected detections list"}), 400
    if len(payload["detections"]) > 100:
        return jsonify({"error": "Too many detections"}), 400
    try:
        frame = base64.b64decode(payload["frame_jpeg"], validate=True)
        fps = float(payload.get("fps", 0.0))
        if not frame.startswith(b"\xff\xd8") or not frame.endswith(b"\xff\xd9") or len(frame) > 1500000:
            raise ValueError("Invalid JPEG")
        if not 0 <= fps <= 240:
            raise ValueError("Invalid FPS")
        detections = []
        for item in payload["detections"]:
            label = str(item["label"]).lower().strip()
            score = float(item["confidence"])
            if label not in {"boots", "helmet", "helmet on", "no boots", "no helmet", "no vest", "person", "vest"} or not 0 <= score <= 1:
                raise ValueError("Invalid detection")
            detections.append((label, score))
    except (KeyError, TypeError, ValueError, binascii.Error):
        return jsonify({"error": "Invalid camera frame or detections"}), 400


    try:
        incoming_workers = payload.get("workers", [])
        if not isinstance(incoming_workers, list) or len(incoming_workers) > 5:
            raise ValueError("Invalid workers")
        worker_labels = {
            0: "Unknown worker",
            1: "Worker 1 - Mark",
            2: "Worker 2 - Hansel",
            3: "Worker 3 - Cesar",
        }
        workers = []
        for item in incoming_workers:
            worker_id = item["id"]
            if type(worker_id) is not int or worker_id not in worker_labels:
                raise ValueError("Invalid worker ID")
            similarity = float(item.get("similarity", 0))
            if not math.isfinite(similarity) or not -1.01 <= similarity <= 1.01:
                raise ValueError("Invalid similarity")
            workers.append({
                "id": worker_id,
                "label": worker_labels[worker_id],
                "similarity": round(similarity, 3),
            })
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Invalid worker data"}), 400

    scores: dict[str, float] = {}
    for label, score in detections:
        scores[label] = max(score, scores.get(label, 0.0))
    # Do not claim PPE compliance when no person is visible.
    person_visible = scores.get("person", 0.0) >= 0.5
    helmet = ppe_status(scores, ("helmet", "helmet on"), "no helmet") if person_visible else "UNKNOWN"
    vest = ppe_status(scores, ("vest",), "no vest") if person_visible else "UNKNOWN"

    with state_lock:
        for kind, old, new, message in (
            ("HELMET", state["helmet_status"], helmet, "Camera detected a person without a helmet"),
            ("VEST", state["vest_status"], vest, "Camera detected a person without a vest"),
        ):
            if new == "MISSING" and old != "MISSING":
                add_alert("PPE", message, "warning")
        state["helmet_status"] = helmet
        state["vest_status"] = vest
        state["detections"] = [f"{label} ({score:.2f})" for label, score in detections]
        state["workers"] = workers
        state["fps"] = round(fps, 1)
        state["camera_frame"] = frame
        state["camera_last_update"] = time.time()
    return jsonify({"status": "ok"})


@app.route("/api/fall/update", methods=["POST"])
def fall_update():
    if not authorized():
        return jsonify({"error": "Unauthorized"}), 401
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Expected motion sample"}), 400
    try:
        axes = [float(payload[key]) for key in ("ax", "ay", "az")]
        event = payload.get("event", "")
        # PDR fields are optional so the USB MPU6050 bridge keeps working.
        pdr = {key: round(float(payload[key]), 2)
               for key in ("x", "y", "heading", "distance") if key in payload}
        if "steps" in payload:
            pdr["steps"] = int(payload["steps"])
        if "walking" in payload:
            pdr["walking"] = payload["walking"] is True
        peak_g = float(payload.get("peak_g", 0))
        severity = int(payload.get("severity", 0))
        if (not all(math.isfinite(value) and abs(value) <= 160 for value in axes)
                or not all(math.isfinite(value) for value in [*pdr.values(), peak_g])
                or event not in ("", "FREE_FALL", "FALL_DETECTED", "FREE_FALL_EXPIRED")):
            raise ValueError("Invalid motion sample")
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Invalid motion sample"}), 400
    with state_lock:
        state["ax"], state["ay"], state["az"] = [round(value, 3) for value in axes]
        state["smv"] = round(math.sqrt(sum(value ** 2 for value in axes)), 3)
        state["motion_has_connected"] = True
        state["motion_last_update"] = time.time()
        state["pdr"].update(pdr)
        if event == "FALL_DETECTED" and not state["fall_active"]:
            state["fall_active"] = True
            message = "Wearable IMU detected free fall followed by impact"
            if peak_g:
                message += f" ({peak_g:.1f} g, severity {severity})"
            if "x" in pdr and "y" in pdr:
                message += f" at {pdr['x']:.1f}, {pdr['y']:.1f} m from start"
            add_alert("FALL", message)
    return jsonify({"status": "ok"})


@app.route("/api/imu", methods=["POST"])
def imu_update():
    """Raw MPU9250 samples from the ESP32 wearable (esp32_safeware.ino)."""
    if not authorized():
        return jsonify({"error": "Unauthorized"}), 401
    payload = request.get_json(silent=True)
    try:
        samples = payload["s"]
        if not isinstance(samples, list) or len(samples) > 200:
            raise ValueError("Bad batch")
        rows = []
        for row in samples:
            if not isinstance(row, list) or len(row) not in (7, 10):
                raise ValueError("Bad sample")
            values = [float(v) for v in row]
            if not all(math.isfinite(v) for v in values) or not 0 <= values[0] < 2**32:
                raise ValueError("Bad sample")
            rows.append(values)
        device = str(payload.get("device", ""))[:32]
        boot = int(payload.get("boot", 0))
        dropped = int(payload.get("dropped", 0))
        imu_ok, mag_ok = payload.get("imu") is True, payload.get("mag") is True
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Invalid IMU batch"}), 400

    with motion_lock:
        falls = motion.ingest(device, boot, imu_ok, mag_ok, dropped, rows)
        accel = [value * 9.80665 for value in motion.last_accel]
        pdr = motion.pdr_summary()
    with state_lock:
        state["ax"], state["ay"], state["az"] = [round(value, 3) for value in accel]
        state["smv"] = round(math.sqrt(sum(value ** 2 for value in accel)), 3)
        state["motion_has_connected"] = True
        state["motion_last_update"] = time.time()
        state["pdr"].update(pdr)
        for peak_g, severity in falls:
            if not state["fall_active"]:
                state["fall_active"] = True
                add_alert("FALL", f"Wearable IMU detected free fall followed by impact ({peak_g:.1f} g, "
                                  f"severity {severity}) at {pdr['x']:.1f}, {pdr['y']:.1f} m from start")
    with state_lock:
        fall_active = bool(state["fall_active"])
    return jsonify({"status": "ok", "fall_active": fall_active})


@app.route("/pdr")
def pdr_page():
    return render_template("pdr.html")


@app.route("/api/pdr")
def pdr_state():
    try:
        start = int(request.args.get("from", 0))
        session = int(request.args.get("s", -1))
    except ValueError:
        return jsonify({"error": "Bad query"}), 400
    with motion_lock:
        return jsonify(motion.snapshot(start, session))


@app.route("/api/pdr/<action>", methods=["POST"])
def pdr_control(action: str):
    body = request.get_json(silent=True) or {}
    with motion_lock:
        if action == "reset":
            motion.reset_pdr()
        elif action == "calibrate":
            motion.start_gyro_cal()
        elif action == "magcal":
            if not motion.start_mag_cal():
                return jsonify({"error": "No magnetometer"}), 400
        elif action == "set":
            try:
                motion.configure(body.get("len"), body.get("snap"), body.get("fuse"))
            except (TypeError, ValueError):
                return jsonify({"error": "Bad setting"}), 400
        else:
            return jsonify({"error": "Unknown action"}), 400
        pdr = motion.pdr_summary()
    with state_lock:
        state["pdr"].update(pdr)
    return jsonify({"status": "ok"})


@app.route("/api/demo/trigger", methods=["POST"])
def trigger_demo_alert():
    kind = (request.get_json(silent=True) or {}).get("type", "")
    with state_lock:
        if kind == "fall":
            state["fall_active"] = True
            if not state["motion_has_connected"]:
                state["smv"] = 24.8
            add_alert("FALL", "Demo fall alert (simulated)")
        elif kind == "temp":
            state["temp_alert"] = True
            state["temp_f"] = 104.2
            state["temp_c"] = 40.1
            add_alert("HEAT", "Worker temperature exceeded 103 F")
        elif kind == "hr":
            state["hr_alert"] = True
            state["hr_bpm"] = 132
            add_alert("HEART RATE", "Heart rate exceeded 120 BPM")
        elif kind == "helmet":
            return jsonify({"error": "PPE is controlled by the camera"}), 400
        elif kind == "vest":
            return jsonify({"error": "PPE is controlled by the camera"}), 400
        else:
            return jsonify({"error": "Unknown demo event"}), 400
    return jsonify({"status": "ok"})


@app.route("/api/acknowledge/<kind>", methods=["POST"])
def acknowledge(kind: str):
    with state_lock:
        if kind == "fall":
            state["fall_active"] = False
        elif kind == "temp":
            state["temp_alert"] = False
            state["temp_f"] = 98.6
            state["temp_c"] = 37.0
        elif kind == "hr":
            state["hr_alert"] = False
            state["hr_bpm"] = 82
        elif kind == "all":
            state["fall_active"] = False
            state["temp_alert"] = False
            state["hr_alert"] = False
            state["temp_f"] = 98.6
            state["temp_c"] = 37.0
            state["hr_bpm"] = 82
        else:
            return jsonify({"error": "Unknown alert"}), 400
    return jsonify({"status": "cleared"})


# Compatibility with the current Raspberry Pi application's routes.
@app.route("/api/acknowledge_fall", methods=["POST"])
def acknowledge_fall():
    return acknowledge("fall")


@app.route("/api/acknowledge_temp", methods=["POST"])
def acknowledge_temp():
    return acknowledge("temp")


@app.route("/api/acknowledge_hr", methods=["POST"])
def acknowledge_hr():
    return acknowledge("hr")


if __name__ == "__main__":
    worker = threading.Thread(target=simulator, daemon=True, name="DEMO-SIMULATOR")
    worker.start()
    print("\nPPE dashboard is running at http://127.0.0.1:5000")
    print("Press Ctrl+C to stop it.\n")
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        stop_event.set()
