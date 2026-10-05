"""Edits for demo_dashboard.py so the ESP32 can post over WiFi.

Apply the five numbered edits below. Everything else in the file stays as is.
"""

# ---------------------------------------------------------------------------
# EDIT 1: add "import os" next to "import math" at the top of the file.
# ---------------------------------------------------------------------------
import os

# ---------------------------------------------------------------------------
# EDIT 2: paste this block right after the hr_history = deque(...) line.
# ---------------------------------------------------------------------------
# Set SAFEWARE_TOKEN in the environment to require X-Safeware-Token on ingest routes.
API_TOKEN = os.environ.get("SAFEWARE_TOKEN", "")
state.update({
    "pdr_x": 0.0, "pdr_y": 0.0, "heading": 0.0, "steps": 0, "distance": 0.0,
    "walking": False, "fall_severity": 0, "fall_peak_g": 0.0, "motion_device": "",
})
pdr_trail: deque[tuple[float, float]] = deque(maxlen=300)


def authorized() -> bool:
    """Localhost (the camera script) is always allowed; remote devices need the token."""
    if not API_TOKEN or request.remote_addr in ("127.0.0.1", "::1"):
        return True
    return request.headers.get("X-Safeware-Token", "") == API_TOKEN


# ---------------------------------------------------------------------------
# EDIT 3: in api_state(), after the line snapshot["hr_history"] = list(hr_history), add:
#         snapshot["pdr_trail"] = list(pdr_trail)
# And in camera_update(), make this the first statement:
#         if not authorized():
#             return jsonify({"error": "Unauthorized"}), 401
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# EDIT 4: replace the whole fall_update() function with this one.
# ---------------------------------------------------------------------------
@app.route("/api/fall/update", methods=["POST"])
def fall_update():
    """Motion sample from the wearable. PDR and severity fields are optional."""
    if not authorized():
        return jsonify({"error": "Unauthorized"}), 401
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Expected motion sample"}), 400
    try:
        axes = [float(payload[key]) for key in ("ax", "ay", "az")]
        event = payload.get("event", "")
        pdr = {key: float(payload.get(key, 0.0)) for key in ("x", "y", "heading", "distance", "peak_g")}
        steps = int(payload.get("steps", 0))
        severity = int(payload.get("severity", 0))
        walking = bool(payload.get("walking", False))
        device = str(payload.get("device", ""))[:32]
        if (not all(math.isfinite(value) and abs(value) <= 160 for value in axes)
                or not all(math.isfinite(value) and abs(value) <= 100000 for value in pdr.values())
                or event not in ("", "FREE_FALL", "FALL_DETECTED", "FREE_FALL_EXPIRED")
                or severity not in (0, 1, 2) or steps < 0):
            raise ValueError("Invalid motion sample")
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Invalid motion sample"}), 400
    with state_lock:
        state["ax"], state["ay"], state["az"] = [round(value, 3) for value in axes]
        state["smv"] = round(math.sqrt(sum(value ** 2 for value in axes)), 3)
        state["motion_has_connected"] = True
        state["motion_last_update"] = time.time()
        state["motion_device"] = device
        state["pdr_x"], state["pdr_y"] = round(pdr["x"], 2), round(pdr["y"], 2)
        state["heading"], state["distance"] = round(pdr["heading"], 1), round(pdr["distance"], 2)
        state["steps"], state["walking"] = steps, walking
        point = (state["pdr_x"], state["pdr_y"])
        if not pdr_trail or pdr_trail[-1] != point:
            pdr_trail.append(point)
        if event == "FALL_DETECTED" and not state["fall_active"]:
            state["fall_active"] = True
            state["fall_severity"] = severity
            state["fall_peak_g"] = round(pdr["peak_g"], 2)
            add_alert("FALL", f"Wearable confirmed a fall (peak {pdr['peak_g']:.1f} g)")
    return jsonify({"status": "ok"})


# ---------------------------------------------------------------------------
# EDIT 5: at the bottom, change app.run(host="127.0.0.1", ...) to host="0.0.0.0".
# Start the server with:  SAFEWARE_TOKEN=change-me python demo_dashboard.py
# ---------------------------------------------------------------------------
