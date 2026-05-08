"""
Unified Sensor Dashboard - Raspberry Pi
Combines: MPU6050 (XYZ/fall) + SIMULATED GPS + YOLOv8 Camera + MLX90614 (temp) + MCP3008/Pulse (HR)
Hardware: Passive buzzer GPIO17, LED GPIO27, Touch sensor GPIO23
Dashboard: served over WiFi via Flask

NOTE: GPS module hardware is currently bypassed -- using simulated GPS data
that random-walks around a construction site near Marietta, GA.
To switch back to real GPS later: restore gps_thread() and remove sim_gps_thread().
"""

import math
import random
import time
import threading
import cv2
import smbus2
import smtplib
from email.mime.text import MIMEText
import RPi.GPIO as GPIO
from flask import Flask, Response, jsonify, render_template_string, request
from mpu6050 import mpu6050
from ultralytics import YOLO
from picamera2 import Picamera2

# ---------------------------------------------------------------------
# EMAIL CONFIG -- loaded from ~/email_config.py on the Pi
# ---------------------------------------------------------------------
try:
    from email_config import EMAIL_SENDER, EMAIL_PASSWORD, EMAIL_RECEIVER
except ImportError:
    EMAIL_SENDER = EMAIL_PASSWORD = EMAIL_RECEIVER = None
    print("[EMAIL] email_config.py not found -- email alerts disabled.")

# ---------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------
MODEL_PATH        = "yolov8n.pt"
CONF              = 0.45
IOU               = 0.45
IMGSZ             = 256
CAM_W, CAM_H      = 480, 360
FREE_FALL_THRESH  = 3.0
IMPACT_THRESH     = 20.0
IMPACT_WINDOW     = 1.5
SAMPLE_RATE       = 0.02
MAX_ALERTS        = 20
TEMP_ALERT_F      = 103.0    # F threshold for heat alert
HR_ALERT_BPM      = 120       # BPM threshold for tachycardia alert
HR_SAMPLE_WINDOW  = 10        # seconds of samples to average BPM over

# --- GPS simulator config ---
SIM_BASE_LAT      = 33.9526   # Marietta, GA
SIM_BASE_LON      = -84.5499
SIM_BASE_ALT      = 310.0     # meters
SIM_SITE_RADIUS_M = 80        # keep worker within this radius
SIM_WALK_SPEED    = 1.2       # m/s (walking pace)
SIM_UPDATE_HZ     = 1.0       # GPS-style 1 Hz

# GPIO pins
PIN_BUZZER  = 17
PIN_LED     = 27
PIN_TOUCH   = 23

# MLX90614 I2C
MLX_ADDR      = 0x5A
MLX_RAM_TOBJ1 = 0x07   # object temperature register

# ---------------------------------------------------------------------
# GPIO SETUP
# ---------------------------------------------------------------------
def gpio_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_BUZZER, GPIO.OUT)
    GPIO.setup(PIN_LED,    GPIO.OUT)
    GPIO.setup(PIN_TOUCH,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

gpio_init()
buzzer_pwm = GPIO.PWM(PIN_BUZZER, 1000)

# ---------------------------------------------------------------------
# SHARED STATE
# ---------------------------------------------------------------------
state_lock = threading.Lock()
state = {
    "ax": 0.0, "ay": 0.0, "az": 0.0, "smv": 0.0,
    "lat": "No Fix", "lon": "No Fix", "alt": "No Fix",
    "fall": False,
    "fall_active": False,
    "helmet_on": False,
    "temp_f": 0.0,
    "temp_c": 0.0,
    "temp_alert": False,
    "alerts": [],
    "temp_alerts": [],
    "hr_bpm": 0,
    "hr_alert": False,
    "hr_alerts": [],
    "detections": [],
    "fps": 0.0,
}

frame_lock  = threading.Lock()
latest_jpeg = None
stop_event  = threading.Event()


# ---------------------------------------------------------------------
# EMAIL ALERT FUNCTION
# ---------------------------------------------------------------------
def send_email(subject, body):
    if not EMAIL_SENDER:
        return
    try:
        msg = MIMEText(body)
        msg["Subject"] = subject
        msg["From"]    = EMAIL_SENDER
        msg["To"]      = EMAIL_RECEIVER
        with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
            server.login(EMAIL_SENDER, EMAIL_PASSWORD)
            server.sendmail(EMAIL_SENDER, EMAIL_RECEIVER, msg.as_string())
        print("[EMAIL] Alert sent successfully.")
    except Exception as e:
        print(f"[EMAIL ERROR] {e}")

# ---------------------------------------------------------------------
# MLX90614 TEMPERATURE THREAD
# ---------------------------------------------------------------------
def read_mlx90614(bus):
    raw    = bus.read_word_data(MLX_ADDR, MLX_RAM_TOBJ1)
    temp_k = raw * 0.02
    temp_c = temp_k - 273.15
    return temp_c

def temp_thread():
    try:
        bus = smbus2.SMBus(1)
    except Exception as e:
        print(f"[TEMP ERROR] Could not open I2C bus: {e}")
        return

    while not stop_event.is_set():
        try:
            temp_c = read_mlx90614(bus)
            temp_f = temp_c * 9.0 / 5.0 + 32.0

            with state_lock:
                state["temp_c"] = round(temp_c, 1)
                state["temp_f"] = round(temp_f, 1)

                if temp_f >= TEMP_ALERT_F and not state["temp_alert"]:
                    state["temp_alert"] = True
                    alert = {
                        "time":   time.strftime("%H:%M:%S"),
                        "temp_f": round(temp_f, 1),
                        "temp_c": round(temp_c, 1),
                        "lat":    state["lat"],
                        "lon":    state["lon"],
                    }
                    state["temp_alerts"].insert(0, alert)
                    if len(state["temp_alerts"]) > MAX_ALERTS:
                        state["temp_alerts"].pop()
                    tf_snap = round(temp_f, 1)
                    tc_snap = round(temp_c, 1)
                    lat_snap2 = state["lat"]
                    lon_snap2 = state["lon"]
                    print(f"[TEMP ALERT] {temp_f:.1f}F exceeds threshold!")
                    t2 = threading.Thread(target=send_email, daemon=True, args=(
                        "HIGH TEMP ALERT -- Worker Overheating",
                        f"Worker temperature exceeded 103F.\n\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')}\nTemp: {tf_snap}F / {tc_snap}C\nLat: {lat_snap2}\nLon: {lon_snap2}\n\nDashboard: http://192.168.1.212:5000"
                    ))
                    t2.start()

        except Exception as e:
            print(f"[TEMP ERROR] {e}")

        time.sleep(1.0)


# ---------------------------------------------------------------------
# HEART RATE THREAD -- MCP3008 + analog pulse sensor on CH0
# ---------------------------------------------------------------------
def hr_thread():
    try:
        import spidev
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 1350000
        spi.mode = 0
    except Exception as e:
        print(f"[HR ERROR] Could not init MCP3008 SPI: {e}")
        return

    def read_mcp3008(channel=0):
        r = spi.xfer2([1, (8 + channel) << 4, 0])
        return ((r[1] & 3) << 8) | r[2]

    print("[HR] Heart rate sensor ready on MCP3008 CH0")

    peak_times = []
    last_val   = 0
    rising     = False
    threshold  = 512

    while not stop_event.is_set():
        try:
            val = read_mcp3008(0)

            if not rising and val > threshold and last_val <= threshold:
                rising = True
                now = time.time()
                peak_times.append(now)
                peak_times = [t for t in peak_times if now - t <= HR_SAMPLE_WINDOW]

            elif rising and val < threshold:
                rising = False

            last_val = val

            bpm = 0
            if len(peak_times) >= 2:
                intervals = [peak_times[i+1] - peak_times[i]
                             for i in range(len(peak_times)-1)]
                avg_interval = sum(intervals) / len(intervals)
                if avg_interval > 0:
                    bpm = int(60.0 / avg_interval)

            with state_lock:
                state["hr_bpm"] = bpm

                if bpm >= HR_ALERT_BPM and not state["hr_alert"]:
                    state["hr_alert"] = True
                    alert = {
                        "time":  time.strftime("%H:%M:%S"),
                        "bpm":   bpm,
                        "lat":   state["lat"],
                        "lon":   state["lon"],
                    }
                    state["hr_alerts"].insert(0, alert)
                    if len(state["hr_alerts"]) > MAX_ALERTS:
                        state["hr_alerts"].pop()
                    print(f"[HR ALERT] {bpm} BPM exceeds threshold!")
                    lat_s = state["lat"]
                    lon_s = state["lon"]
                    t_email = threading.Thread(target=send_email, daemon=True, args=(
                        "HIGH HEART RATE ALERT -- Tachycardia Detected",
                        f"Worker heart rate exceeded 120 BPM.\n\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')}\nBPM: {bpm}\nLat: {lat_s}\nLon: {lon_s}\n\nDashboard: http://192.168.1.212:5000"
                    ))
                    t_email.start()

                elif bpm < HR_ALERT_BPM and state["hr_alert"] and bpm > 0:
                    state["hr_alert"] = False

        except Exception as e:
            print(f"[HR ERROR] {e}")

        time.sleep(0.02)

# ---------------------------------------------------------------------
# BUZZER + LED THREAD
# ---------------------------------------------------------------------
def alert_hardware_thread():
    gpio_init()
    buzzer_on = False
    while not stop_event.is_set():
        with state_lock:
            active = state["fall_active"] or state["temp_alert"] or state["hr_alert"]

        if active:
            GPIO.output(PIN_LED, GPIO.HIGH)
            if not buzzer_on:
                buzzer_pwm.start(50)
                buzzer_on = True
            time.sleep(0.3)
            GPIO.output(PIN_LED, GPIO.LOW)
            time.sleep(0.3)
        else:
            if buzzer_on:
                buzzer_pwm.stop()
                buzzer_on = False
            GPIO.output(PIN_LED,    GPIO.LOW)
            GPIO.output(PIN_BUZZER, GPIO.LOW)
            time.sleep(0.1)

# ---------------------------------------------------------------------
# TOUCH SENSOR THREAD
# ---------------------------------------------------------------------
def touch_thread():
    gpio_init()
    while not stop_event.is_set():
        reading = GPIO.input(PIN_TOUCH)
        with state_lock:
            state["helmet_on"] = bool(reading)
        time.sleep(0.1)

# ---------------------------------------------------------------------
# IMU + FALL DETECTION THREAD
# ---------------------------------------------------------------------
def imu_thread():
    sensor = mpu6050(0x68)
    in_freefall   = False
    freefall_time = None

    while not stop_event.is_set():
        try:
            a  = sensor.get_accel_data()
            ax, ay, az = a["x"], a["y"], a["z"]
            smv = math.sqrt(ax**2 + ay**2 + az**2)
            fall_now = False

            if not in_freefall and smv < FREE_FALL_THRESH:
                in_freefall   = True
                freefall_time = time.time()

            elif in_freefall:
                if smv > IMPACT_THRESH:
                    fall_now    = True
                    in_freefall = False
                    with state_lock:
                        alert = {
                            "time": time.strftime("%H:%M:%S"),
                            "lat":  state["lat"],
                            "lon":  state["lon"],
                            "smv":  round(smv, 2),
                        }
                        state["alerts"].insert(0, alert)
                        if len(state["alerts"]) > MAX_ALERTS:
                            state["alerts"].pop()
                        state["fall_active"] = True
                        lat_snap = state["lat"]
                        lon_snap = state["lon"]
                        smv_snap = round(smv, 2)
                    t = threading.Thread(target=send_email, daemon=True, args=(
                        "FALL DETECTED -- Worker Down",
                        f"A fall was detected.\n\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')}\nSMV: {smv_snap} m/s^2\nLat: {lat_snap}\nLon: {lon_snap}\n\nDashboard: http://192.168.1.212:5000"
                    ))
                    t.start()
                elif time.time() - freefall_time > IMPACT_WINDOW:
                    in_freefall = False

            with state_lock:
                state["ax"]   = round(ax, 3)
                state["ay"]   = round(ay, 3)
                state["az"]   = round(az, 3)
                state["smv"]  = round(smv, 3)
                state["fall"] = fall_now

        except Exception as e:
            print(f"[IMU ERROR] {e}")

        time.sleep(SAMPLE_RATE)

# ---------------------------------------------------------------------
# SIMULATED GPS THREAD -- replaces real NEO-6M / gpsd while hardware is down
# Random walk around SIM_BASE_LAT / SIM_BASE_LON, stays within site radius.
# ---------------------------------------------------------------------
def sim_gps_thread():
    METERS_PER_DEG_LAT = 111_111.0
    METERS_PER_DEG_LON = 111_111.0 * math.cos(math.radians(SIM_BASE_LAT))

    lat = SIM_BASE_LAT
    lon = SIM_BASE_LON
    alt = SIM_BASE_ALT
    heading = random.uniform(0, 360)
    speed = 0.0
    has_fix = False
    last = time.time()

    # warm-up: pretend we're acquiring fix for a few seconds
    print("[GPS-SIM] Warming up (simulating fix acquisition)...")
    time.sleep(3.0)
    has_fix = True
    print(f"[GPS-SIM] 3D fix acquired at {SIM_BASE_LAT:.4f}, {SIM_BASE_LON:.4f}")

    while not stop_event.is_set():
        now = time.time()
        dt  = now - last
        last = now

        # heading drift (worker turns occasionally)
        heading = (heading + random.gauss(0, 15) * dt) % 360

        # speed wobbles around walking pace, sometimes pauses
        target_speed = SIM_WALK_SPEED if random.random() > 0.1 else 0.0
        speed += (target_speed - speed) * 0.3
        speed = max(0.0, min(speed, 2.0))

        # convert heading + speed to lat/lon delta
        dist_m = speed * dt
        dlat_m = dist_m * math.cos(math.radians(heading))
        dlon_m = dist_m * math.sin(math.radians(heading))

        new_lat = lat + dlat_m / METERS_PER_DEG_LAT
        new_lon = lon + dlon_m / METERS_PER_DEG_LON

        # stay inside the site -- if we'd leave, reverse heading
        dlat_from_base = (new_lat - SIM_BASE_LAT) * METERS_PER_DEG_LAT
        dlon_from_base = (new_lon - SIM_BASE_LON) * METERS_PER_DEG_LON
        dist_from_base = math.sqrt(dlat_from_base**2 + dlon_from_base**2)

        if dist_from_base > SIM_SITE_RADIUS_M:
            # turn back toward base
            heading = math.degrees(math.atan2(
                (SIM_BASE_LON - lon) * METERS_PER_DEG_LON,
                (SIM_BASE_LAT - lat) * METERS_PER_DEG_LAT
            )) % 360
        else:
            lat = new_lat
            lon = new_lon

        # altitude wobble
        alt = max(SIM_BASE_ALT - 5, min(SIM_BASE_ALT + 5, alt + random.gauss(0, 0.3)))

        # very rare brief fix loss
        has_fix = random.random() > 0.002

        with state_lock:
            if has_fix:
                state["lat"] = round(lat, 6)
                state["lon"] = round(lon, 6)
                state["alt"] = round(alt, 1)
            else:
                state["lat"] = "No Fix"
                state["lon"] = "No Fix"
                state["alt"] = "No Fix"

        time.sleep(1.0 / SIM_UPDATE_HZ)

# ---------------------------------------------------------------------
# CAMERA + YOLO THREAD
# ---------------------------------------------------------------------
def camera_thread():
    global latest_jpeg

    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "RGB888"},
        controls={"FrameRate": 30}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(0.8)
    print(f"[CAMERA] Ready ({CAM_W}x{CAM_H})")

    model = YOLO(MODEL_PATH)
    font  = cv2.FONT_HERSHEY_SIMPLEX
    fps   = 0.0
    fc    = 0
    t_ref = time.time()

    try:
        while not stop_event.is_set():
            rgb   = cam.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            results = model.predict(
                source=frame, conf=CONF, iou=IOU,
                imgsz=IMGSZ, verbose=False
            )

            tags = []
            for box in results[0].boxes:
                cls_id     = int(box.cls[0])
                confidence = float(box.conf[0])
                label      = f"{model.names[cls_id]} {confidence:.2f}"
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                tags.append(f"{model.names[cls_id]}({confidence:.2f})")
                cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)
                (tw,th),_ = cv2.getTextSize(label, font, 0.5, 1)
                cv2.rectangle(frame,(x1,y1-th-6),(x1+tw+4,y1),(0,200,0),-1)
                cv2.putText(frame, label,(x1+2,y1-4),font,0.5,(0,0,0),1,cv2.LINE_AA)

            fc += 1
            if fc % 10 == 0:
                fps   = 10.0 / (time.time() - t_ref + 1e-6)
                t_ref = time.time()
                fc    = 0

            cv2.putText(frame, f"FPS:{fps:.1f}", (8,24),
                        font, 0.7, (0,255,255), 2, cv2.LINE_AA)

            with state_lock:
                state["detections"] = tags
                state["fps"]        = round(fps, 1)

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with frame_lock:
                    latest_jpeg = buf.tobytes()

    finally:
        cam.stop()
        print("[CAMERA] Stopped.")

# ---------------------------------------------------------------------
# FLASK APP
# ---------------------------------------------------------------------
app = Flask(__name__)

DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Sensor Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow:wght@300;600;800&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:     #080c10;
    --panel:  #0d1520;
    --border: #1a2d44;
    --accent: #00e5ff;
    --green:  #39ff14;
    --red:    #ff1744;
    --yellow: #ffd600;
    --orange: #ff6d00;
    --text:   #c8d8e8;
    --dim:    #445566;
    --mono:   'Share Tech Mono', monospace;
    --sans:   'Barlow', sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: var(--sans); min-height: 100vh; padding: 16px; }
  body::before {
    content: ''; position: fixed; inset: 0;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,0,0,0.06) 2px, rgba(0,0,0,0.06) 4px);
    pointer-events: none; z-index: 999;
  }
  header { display:flex; align-items:center; gap:14px; margin-bottom:18px; border-bottom:1px solid var(--border); padding-bottom:12px; }
  header .dot { width:10px; height:10px; border-radius:50%; background:var(--green); animation:pulse 1.4s infinite; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }
  header h1 { font-family:var(--sans); font-weight:800; font-size:1.1rem; letter-spacing:0.18em; text-transform:uppercase; color:var(--accent); }
  header .sub { font-family:var(--mono); font-size:0.7rem; color:var(--dim); margin-left:auto; }

  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr 1fr 320px;
    grid-template-rows: auto auto auto;
    gap: 12px;
  }

  .panel { background:var(--panel); border:1px solid var(--border); border-radius:6px; padding:16px; position:relative; overflow:hidden; }
  .panel::after { content:''; position:absolute; top:0; left:0; right:0; height:2px; background:linear-gradient(90deg,var(--accent),transparent); }
  .panel-label { font-family:var(--mono); font-size:0.65rem; letter-spacing:0.2em; color:var(--dim); text-transform:uppercase; margin-bottom:14px; }

  /* IMU */
  .axis-row { display:flex; align-items:center; gap:10px; margin-bottom:10px; }
  .axis-label { font-family:var(--mono); font-size:0.75rem; color:var(--dim); width:22px; }
  .axis-bar-wrap { flex:1; height:8px; background:#0a1520; border-radius:4px; overflow:hidden; }
  .axis-bar { height:100%; border-radius:4px; transition:width 0.15s ease; }
  .bar-x { background:var(--accent); } .bar-y { background:var(--green); } .bar-z { background:var(--yellow); }
  .axis-val { font-family:var(--mono); font-size:0.8rem; width:64px; text-align:right; }
  .smv-row { margin-top:14px; padding-top:10px; border-top:1px solid var(--border); display:flex; justify-content:space-between; align-items:center; }
  .smv-label { font-family:var(--mono); font-size:0.65rem; color:var(--dim); }
  .smv-val { font-family:var(--mono); font-size:1.2rem; color:var(--accent); }

  /* Temperature */
  .temp-main { display:flex; align-items:baseline; gap:10px; margin-bottom:4px; }
  .temp-big { font-family:var(--mono); font-size:2.6rem; font-weight:700; line-height:1; transition: color 0.4s; }
  .temp-unit { font-family:var(--mono); font-size:1rem; color:var(--dim); }
  .temp-sub { font-family:var(--mono); font-size:0.75rem; color:var(--dim); margin-bottom:12px; }
  .temp-bar-wrap { width:100%; height:10px; background:#0a1520; border-radius:5px; overflow:hidden; margin-bottom:5px; }
  .temp-bar { height:100%; border-radius:5px; transition: width 0.4s ease, background 0.4s ease; }
  .temp-range { display:flex; justify-content:space-between; font-family:var(--mono); font-size:0.58rem; color:var(--dim); }

  /* Heart Rate */
  .hr-main { display:flex; align-items:baseline; gap:10px; margin-bottom:4px; }
  .hr-big { font-family:var(--mono); font-size:2.6rem; font-weight:700; line-height:1; transition: color 0.4s; }
  .hr-unit { font-family:var(--mono); font-size:1rem; color:var(--dim); }
  .hr-sub { font-family:var(--mono); font-size:0.75rem; color:var(--dim); margin-bottom:12px; }
  .hr-bar-wrap { width:100%; height:10px; background:#0a1520; border-radius:5px; overflow:hidden; margin-bottom:5px; }
  .hr-bar { height:100%; border-radius:5px; transition: width 0.4s ease, background 0.4s ease; }
  .hr-range { display:flex; justify-content:space-between; font-family:var(--mono); font-size:0.58rem; color:var(--dim); }
  .hr-beat { display:inline-block; animation: heartbeat 0.6s ease-in-out infinite; }
  @keyframes heartbeat { 0%,100%{transform:scale(1)} 50%{transform:scale(1.3)} }
  #hr-overlay { display:none; position:fixed; inset:0; pointer-events:none; z-index:100; background: rgba(255,23,68,0.12); animation: flashbg 0.6s infinite alternate; }
  #hr-banner { display:none; border-radius:6px; padding:12px 16px; margin-top:10px; background: #cc0033; color:#fff; animation: flashbang 0.4s infinite alternate; }

  /* Alert overlays */
  #fall-overlay, #temp-overlay {
    display:none; position:fixed; inset:0; pointer-events:none; z-index:100;
    animation: flashbg 0.6s infinite alternate;
  }
  #fall-overlay { background: rgba(255,23,68,0.15); }
  #temp-overlay { background: rgba(255,109,0,0.15); }
  @keyframes flashbg { from{opacity:1} to{opacity:0} }

  #fall-banner, #temp-banner {
    display:none; border-radius:6px; padding:12px 16px; margin-top:10px;
    animation: flashbang 0.4s infinite alternate;
  }
  #fall-banner { background: var(--red); color:#fff; }
  #temp-banner { background: var(--orange); color:#fff; }
  @keyframes flashbang { from{opacity:1} to{opacity:0.5} }
  .fall-title { font-weight:800; font-size:0.85rem; letter-spacing:0.18em; margin-bottom:8px; }
  .fall-clear-btn {
    background:#fff; border:none; border-radius:4px; font-family:var(--mono);
    font-size:0.72rem; font-weight:700; letter-spacing:0.08em;
    padding:6px 14px; cursor:pointer; width:100%; margin-top:4px;
  }
  #fall-banner .fall-clear-btn { color:var(--red); }
  #temp-banner .fall-clear-btn { color:var(--orange); }
  .fall-clear-btn:hover { background:#f0f0f0; }

  /* Helmet */
  .helmet-row { display:flex; align-items:center; gap:10px; margin-top:12px; padding-top:12px; border-top:1px solid var(--border); }
  .helmet-icon { font-size:1.2rem; }
  .helmet-label { font-family:var(--mono); font-size:0.65rem; color:var(--dim); }
  .helmet-status { font-family:var(--mono); font-size:0.85rem; margin-left:auto; font-weight:700; }
  .helmet-on  { color:var(--green); }
  .helmet-off { color:var(--red); animation:flashbang 0.8s infinite alternate; }

  /* GPS */
  .gps-row { display:flex; justify-content:space-between; align-items:baseline; margin-bottom:10px; padding-bottom:10px; border-bottom:1px solid var(--border); }
  .gps-key { font-family:var(--mono); font-size:0.65rem; color:var(--dim); }
  .gps-val { font-family:var(--mono); font-size:0.9rem; color:var(--green); }

  /* Camera */
  .cam-panel { grid-column:5; grid-row:1/4; display:flex; flex-direction:column; gap:10px; }
  #cam-feed { width:100%; border-radius:4px; display:block; border:1px solid var(--border); }
  .fps-badge { font-family:var(--mono); font-size:0.7rem; color:var(--dim); }
  .fps-badge span { color:var(--yellow); }
  .detections-list { display:flex; flex-wrap:wrap; gap:6px; margin-top:6px; }
  .det-tag { font-family:var(--mono); font-size:0.65rem; background:#0a1e10; border:1px solid var(--green); color:var(--green); padding:2px 8px; border-radius:3px; }

  /* Status bar */
  .status-panel { grid-column:1/5; display:flex; gap:16px; align-items:center; padding:10px 16px; }
  .status-item { display:flex; align-items:center; gap:8px; }
  .status-dot { width:8px; height:8px; border-radius:50%; }
  .s-green  { background:var(--green);  box-shadow:0 0 6px var(--green); }
  .s-red    { background:var(--red);    box-shadow:0 0 6px var(--red);    animation:flashbang 0.6s infinite alternate; }
  .s-orange { background:var(--orange); box-shadow:0 0 6px var(--orange); animation:flashbang 0.6s infinite alternate; }
  .s-dim    { background:var(--dim); }
  .status-label { font-family:var(--mono); font-size:0.65rem; color:var(--dim); }

  /* Alert log */
  .alerts-panel { grid-column:1/5; }
  .log-tabs { display:flex; gap:8px; margin-bottom:12px; }
  .tab-btn { font-family:var(--mono); font-size:0.65rem; letter-spacing:0.15em; padding:4px 14px; border-radius:3px; border:1px solid var(--border); background:transparent; color:var(--dim); cursor:pointer; }
  .tab-btn.active { background:var(--accent); color:#000; border-color:var(--accent); }
  .alert-table { width:100%; border-collapse:collapse; font-family:var(--mono); font-size:0.72rem; }
  .alert-table th { color:var(--dim); font-weight:400; text-align:left; padding:4px 8px; border-bottom:1px solid var(--border); }
  .alert-table td { padding:5px 8px; border-bottom:1px solid #111d2a; color:var(--text); }
  .alert-table tr:first-child td { color:var(--red); }
  .alert-table.temp-log tr:first-child td { color:var(--orange); }
  .no-alerts { font-family:var(--mono); font-size:0.72rem; color:var(--dim); padding:10px 0; }
</style>
</head>
<body>

<div id="fall-overlay"></div>
<div id="temp-overlay"></div>
<div id="hr-overlay"></div>

<header>
  <div class="dot"></div>
  <h1>Sensor Dashboard</h1>
  <div class="sub" id="clock">--:--:--</div>
</header>

<div class="grid">

  <!-- IMU Panel -->
  <div class="panel">
    <div class="panel-label">// IMU &mdash; MPU6050</div>
    <div class="axis-row">
      <div class="axis-label">X</div>
      <div class="axis-bar-wrap"><div class="axis-bar bar-x" id="bar-x" style="width:50%"></div></div>
      <div class="axis-val" id="val-x">0.000</div>
    </div>
    <div class="axis-row">
      <div class="axis-label">Y</div>
      <div class="axis-bar-wrap"><div class="axis-bar bar-y" id="bar-y" style="width:50%"></div></div>
      <div class="axis-val" id="val-y">0.000</div>
    </div>
    <div class="axis-row">
      <div class="axis-label">Z</div>
      <div class="axis-bar-wrap"><div class="axis-bar bar-z" id="bar-z" style="width:50%"></div></div>
      <div class="axis-val" id="val-z">9.800</div>
    </div>
    <div class="smv-row">
      <div class="smv-label">SIGNAL MAGNITUDE VECTOR</div>
      <div class="smv-val" id="val-smv">9.800</div>
    </div>
    <div id="fall-banner">
      <div class="fall-title">&#9888; FALL DETECTED &mdash; WORKER DOWN</div>
      <button class="fall-clear-btn" onclick="acknowledgeFall()">
        &#10003; &nbsp;WORKER HAS BEEN ATTENDED TO &mdash; CLEAR ALERT
      </button>
    </div>
    <div class="helmet-row">
      <div class="helmet-icon">&#9981;</div>
      <div class="helmet-label">HELMET STATUS</div>
      <div class="helmet-status" id="helmet-status">--</div>
    </div>
  </div>

  <!-- Temperature Panel -->
  <div class="panel">
    <div class="panel-label">// TEMP &mdash; MLX90614</div>
    <div class="temp-main">
      <div class="temp-big" id="temp-f-big">--.-</div>
      <div class="temp-unit">&deg;F</div>
    </div>
    <div class="temp-sub" id="temp-c-sub">-- &deg;C</div>
    <div class="temp-bar-wrap">
      <div class="temp-bar" id="temp-bar" style="width:0%"></div>
    </div>
    <div class="temp-range">
      <span>90&deg;F</span><span>96.8 normal</span><span>103 &#9888;</span><span>110&deg;F</span>
    </div>
    <div id="temp-banner" style="margin-top:12px;">
      <div class="fall-title">&#127777; HIGH TEMP &mdash; HEAT ALERT</div>
      <button class="fall-clear-btn" onclick="acknowledgeTemp()">
        &#10003; &nbsp;WORKER HAS BEEN CHECKED &mdash; CLEAR ALERT
      </button>
    </div>
  </div>

  <!-- GPS Panel -->
  <div class="panel">
    <div class="panel-label">// GPS &mdash; NEO-6M (SIM)</div>
    <div class="gps-row">
      <div class="gps-key">LATITUDE</div>
      <div class="gps-val" id="gps-lat">Acquiring...</div>
    </div>
    <div class="gps-row">
      <div class="gps-key">LONGITUDE</div>
      <div class="gps-val" id="gps-lon">Acquiring...</div>
    </div>
    <div class="gps-row" style="border:none;margin:0;padding:0">
      <div class="gps-key">ALTITUDE</div>
      <div class="gps-val" id="gps-alt">Acquiring...</div>
    </div>
  </div>


  <!-- Heart Rate Panel -->
  <div class="panel">
    <div class="panel-label">// HEART RATE &mdash; PULSE SENSOR</div>
    <div class="hr-main">
      <span class="hr-beat">&#10084;</span>&nbsp;
      <div class="hr-big" id="hr-big">--</div>
      <div class="hr-unit">BPM</div>
    </div>
    <div class="hr-sub">Beats per minute</div>
    <div class="hr-bar-wrap">
      <div class="hr-bar" id="hr-bar" style="width:0%"></div>
    </div>
    <div class="hr-range">
      <span>40</span><span>60 resting</span><span>100 normal max</span><span>120 &#9888;</span><span>160</span>
    </div>
    <div id="hr-banner" style="margin-top:12px;">
      <div class="fall-title">&#10084; HIGH HEART RATE &mdash; TACHYCARDIA</div>
      <button class="fall-clear-btn" onclick="acknowledgeHR()" style="color:#cc0033;">
        &#10003; &nbsp;WORKER HAS BEEN CHECKED &mdash; CLEAR ALERT
      </button>
    </div>
  </div>

  <!-- Camera Panel -->
  <div class="panel cam-panel">
    <div class="panel-label">// CAMERA &mdash; YOLOV8</div>
    <img id="cam-feed" src="/video_feed" alt="Camera Feed">
    <div class="fps-badge">FPS: <span id="fps-val">--</span></div>
    <div class="detections-list" id="det-list"></div>
  </div>

  <!-- Status Bar -->
  <div class="panel status-panel">
    <div class="status-item">
      <div class="status-dot" id="dot-imu"></div>
      <div class="status-label">IMU</div>
    </div>
    <div class="status-item">
      <div class="status-dot" id="dot-gps"></div>
      <div class="status-label">GPS</div>
    </div>
    <div class="status-item">
      <div class="status-dot" id="dot-helmet"></div>
      <div class="status-label">HELMET</div>
    </div>
    <div class="status-item">
      <div class="status-dot" id="dot-temp"></div>
      <div class="status-label">TEMP</div>
    </div>
    <div class="status-item">
      <div class="status-dot" id="dot-hr"></div>
      <div class="status-label">HEART RATE</div>
    </div>
    <div class="status-item">
      <div class="status-dot" id="dot-fall"></div>
      <div class="status-label">FALL ALERT</div>
    </div>
  </div>

  <!-- Alert Log -->
  <div class="panel alerts-panel">
    <div class="log-tabs">
      <button class="tab-btn active" onclick="showTab('fall')">FALL LOG</button>
      <button class="tab-btn" onclick="showTab('temp')">TEMP LOG</button>
      <button class="tab-btn" onclick="showTab('hr')">HR LOG</button>
    </div>
    <div id="fall-log-body"><div class="no-alerts">No fall events recorded.</div></div>
    <div id="temp-log-body" style="display:none"><div class="no-alerts">No temperature alerts recorded.</div></div>
    <div id="hr-log-body" style="display:none"><div class="no-alerts">No heart rate alerts recorded.</div></div>
  </div>

</div>

<script>
  function showTab(tab) {
    document.getElementById('fall-log-body').style.display = tab === 'fall' ? '' : 'none';
    document.getElementById('temp-log-body').style.display = tab === 'temp' ? '' : 'none';
    document.getElementById('hr-log-body').style.display   = tab === 'hr'   ? '' : 'none';
    document.querySelectorAll('.tab-btn').forEach((b,i) => {
      b.classList.toggle('active',
        (i===0&&tab==='fall')||(i===1&&tab==='temp')||(i===2&&tab==='hr'));
    });
  }

  setInterval(() => {
    document.getElementById('clock').textContent =
      new Date().toLocaleTimeString('en-US', {hour12: false});
  }, 1000);

  function toBar(v) { return ((Math.min(Math.max(v,-20),20)+20)/40*100).toFixed(1)+'%'; }
  function toTempBar(f) { return Math.min(Math.max((f-90)/20*100,0),100).toFixed(1)+'%'; }
  function toHRBar(bpm)  { return Math.min(Math.max((bpm-40)/120*100,0),100).toFixed(1)+'%'; }
  function hrColor(bpm) {
    if (bpm >= 120) return '#ff1744';
    if (bpm >= 100) return '#ffd600';
    return '#39ff14';
  }
  function tempColor(f) {
    if (f >= 103) return '#ff6d00';
    if (f >= 99)  return '#ffd600';
    return '#39ff14';
  }

  async function acknowledgeFall() { await fetch('/api/acknowledge_fall', {method:'POST'}); }
  async function acknowledgeTemp() { await fetch('/api/acknowledge_temp', {method:'POST'}); }
  async function acknowledgeHR()   { await fetch('/api/acknowledge_hr',   {method:'POST'}); }

  async function poll() {
    try {
      const r = await fetch('/api/state');
      const d = await r.json();

      // IMU
      document.getElementById('val-x').textContent   = d.ax.toFixed(3);
      document.getElementById('val-y').textContent   = d.ay.toFixed(3);
      document.getElementById('val-z').textContent   = d.az.toFixed(3);
      document.getElementById('val-smv').textContent = d.smv.toFixed(3);
      document.getElementById('bar-x').style.width   = toBar(d.ax);
      document.getElementById('bar-y').style.width   = toBar(d.ay);
      document.getElementById('bar-z').style.width   = toBar(d.az);

      // Fall
      document.getElementById('fall-banner').style.display  = d.fall_active ? 'block' : 'none';
      document.getElementById('fall-overlay').style.display = d.fall_active ? 'block' : 'none';

      // Temp
      const tf = d.temp_f;
      const tempBig = document.getElementById('temp-f-big');
      tempBig.textContent = tf.toFixed(1);
      tempBig.style.color = tempColor(tf);
      document.getElementById('temp-c-sub').textContent = d.temp_c.toFixed(1) + ' \u00B0C';
      const bar = document.getElementById('temp-bar');
      bar.style.width      = toTempBar(tf);
      bar.style.background = tempColor(tf);
      document.getElementById('temp-banner').style.display  = d.temp_alert ? 'block' : 'none';
      document.getElementById('temp-overlay').style.display = d.temp_alert ? 'block' : 'none';

      // Heart Rate
      const bpm = d.hr_bpm;
      const hrBig = document.getElementById('hr-big');
      hrBig.textContent = bpm > 0 ? bpm : '--';
      hrBig.style.color = bpm > 0 ? hrColor(bpm) : '#445566';
      const hrBar = document.getElementById('hr-bar');
      hrBar.style.width      = toHRBar(bpm);
      hrBar.style.background = hrColor(bpm);
      document.getElementById('hr-banner').style.display  = d.hr_alert ? 'block' : 'none';
      document.getElementById('hr-overlay').style.display = d.hr_alert ? 'block' : 'none';

      // Helmet
      const hEl = document.getElementById('helmet-status');
      hEl.textContent = d.helmet_on ? 'ON' : 'OFF';
      hEl.className   = 'helmet-status ' + (d.helmet_on ? 'helmet-on' : 'helmet-off');

      // GPS
      document.getElementById('gps-lat').textContent = d.lat;
      document.getElementById('gps-lon').textContent = d.lon;
      document.getElementById('gps-alt').textContent = d.alt !== 'No Fix' ? d.alt + ' m' : 'No Fix';

      // Camera
      document.getElementById('fps-val').textContent = d.fps;
      document.getElementById('det-list').innerHTML  = d.detections.length
        ? d.detections.map(t => `<span class="det-tag">${t}</span>`).join('') : '';

      // Status dots
      const dot = (id, cls) => { document.getElementById(id).className = 'status-dot ' + cls; };
      dot('dot-imu',    d.smv > 0          ? 's-green'  : 's-dim');
      dot('dot-gps',    d.lat !== 'No Fix' ? 's-green'  : 's-dim');
      dot('dot-helmet', d.helmet_on        ? 's-green'  : 's-dim');
      dot('dot-temp',   d.temp_alert       ? 's-orange' : 's-green');
      dot('dot-hr',     d.hr_alert         ? 's-red'    : (d.hr_bpm > 0 ? 's-green' : 's-dim'));
      dot('dot-fall',   d.fall_active      ? 's-red'    : 's-dim');

      // HR log
      document.getElementById('hr-log-body').innerHTML = !d.hr_alerts.length
        ? '<div class="no-alerts">No heart rate alerts recorded.</div>'
        : `<table class="alert-table temp-log"><thead><tr>
            <th>TIME</th><th>BPM</th><th>LAT</th><th>LON</th>
           </tr></thead><tbody>${d.hr_alerts.map(a=>
            `<tr><td>${a.time}</td><td>${a.bpm}</td><td>${a.lat}</td><td>${a.lon}</td></tr>`
           ).join('')}</tbody></table>`;

      // Fall log
      document.getElementById('fall-log-body').innerHTML = !d.alerts.length
        ? '<div class="no-alerts">No fall events recorded.</div>'
        : `<table class="alert-table"><thead><tr>
            <th>TIME</th><th>LAT</th><th>LON</th><th>SMV (m/s\u00B2)</th>
           </tr></thead><tbody>${d.alerts.map(a=>
            `<tr><td>${a.time}</td><td>${a.lat}</td><td>${a.lon}</td><td>${a.smv}</td></tr>`
           ).join('')}</tbody></table>`;

      // Temp log
      document.getElementById('temp-log-body').innerHTML = !d.temp_alerts.length
        ? '<div class="no-alerts">No temperature alerts recorded.</div>'
        : `<table class="alert-table temp-log"><thead><tr>
            <th>TIME</th><th>\u00B0F</th><th>\u00B0C</th><th>LAT</th><th>LON</th>
           </tr></thead><tbody>${d.temp_alerts.map(a=>
            `<tr><td>${a.time}</td><td>${a.temp_f}</td><td>${a.temp_c}</td><td>${a.lat}</td><td>${a.lon}</td></tr>`
           ).join('')}</tbody></table>`;

    } catch(e) { /* silently retry */ }
  }

  setInterval(poll, 500);
  poll();
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)

@app.route("/api/state")
def api_state():
    with state_lock:
        return jsonify(dict(state))

@app.route("/api/acknowledge_fall", methods=["POST"])
def acknowledge_fall():
    with state_lock:
        state["fall_active"] = False
    print("[ACK] Fall acknowledged.")
    return jsonify({"status": "cleared"})

@app.route("/api/acknowledge_temp", methods=["POST"])
def acknowledge_temp():
    with state_lock:
        state["temp_alert"] = False
    print("[ACK] Temp alert acknowledged.")
    return jsonify({"status": "cleared"})

def mjpeg_generator():
    global latest_jpeg
    while True:
        with frame_lock:
            frame = latest_jpeg
        if frame:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)

@app.route("/api/acknowledge_hr", methods=["POST"])
def acknowledge_hr():
    with state_lock:
        state["hr_alert"] = False
    print("[ACK] HR alert acknowledged.")
    return jsonify({"status": "cleared"})

@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------
if __name__ == "__main__":
    threads = [
        threading.Thread(target=imu_thread,            daemon=True, name="IMU"),
        threading.Thread(target=sim_gps_thread,        daemon=True, name="GPS-SIM"),
        threading.Thread(target=camera_thread,         daemon=True, name="CAM"),
        threading.Thread(target=temp_thread,           daemon=True, name="TEMP"),
        threading.Thread(target=hr_thread,             daemon=True, name="HR"),
        threading.Thread(target=alert_hardware_thread, daemon=True, name="ALERT"),
        threading.Thread(target=touch_thread,          daemon=True, name="TOUCH"),
    ]

    for t in threads:
        t.start()
        print(f"[BOOT] {t.name} thread started")

    print(f"\n[DASHBOARD] http://192.168.1.212:5000\n")

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        time.sleep(0.6)
        try:
            buzzer_pwm.stop()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
        print("\n[INFO] GPIO cleaned up. Shutdown.")
