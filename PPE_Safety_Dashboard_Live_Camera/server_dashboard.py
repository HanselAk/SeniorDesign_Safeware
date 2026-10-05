"""Dashboard with real YOLO camera results and simulated wearable sensors."""

from __future__ import annotations

import math
import base64
import binascii
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
    "temp_alert": False,
    "hr_alert": False,
    "detections": [],
    "fps": 0.0,
    "camera_last_update": 0.0,
    "camera_frame": None,
    "alerts": [],
    "last_update": time.time(),
}

temp_history: deque[float] = deque([98.2] * HISTORY_POINTS, maxlen=HISTORY_POINTS)
hr_history: deque[int] = deque([78] * HISTORY_POINTS, maxlen=HISTORY_POINTS)


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
        snapshot["camera_online"] = camera_online()
        snapshot["motion_online"] = (state["motion_has_connected"] and
                                     time.time() - state["motion_last_update"] < 3)
        if not snapshot["camera_online"]:
            snapshot["helmet_status"] = "UNKNOWN"
            snapshot["vest_status"] = "UNKNOWN"
            snapshot["detections"] = []
            snapshot["fps"] = 0.0
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
        state["fps"] = round(fps, 1)
        state["camera_frame"] = frame
        state["camera_last_update"] = time.time()
    return jsonify({"status": "ok"})


@app.route("/api/fall/update", methods=["POST"])
def fall_update():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Expected motion sample"}), 400
    try:
        axes = [float(payload[key]) for key in ("ax", "ay", "az")]
        event = payload.get("event", "")
        if (not all(math.isfinite(value) and abs(value) <= 160 for value in axes)
                or event not in ("", "FREE_FALL", "FALL_DETECTED", "FREE_FALL_EXPIRED")):
            raise ValueError("Invalid motion sample")
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Invalid motion sample"}), 400
    with state_lock:
        state["ax"], state["ay"], state["az"] = [round(value, 3) for value in axes]
        state["smv"] = round(math.sqrt(sum(value ** 2 for value in axes)), 3)
        state["motion_has_connected"] = True
        state["motion_last_update"] = time.time()
        if event == "FALL_DETECTED" and not state["fall_active"]:
            state["fall_active"] = True
            add_alert("FALL", "MPU6050 detected free fall followed by impact")
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
        app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)
    finally:
        stop_event.set()
