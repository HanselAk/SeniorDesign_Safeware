"""
Unified Sensor Dashboard - Raspberry Pi
Combines: MPU6050 (XYZ/fall) + REAL GPS (NEO-6M) + YOLOv8 Camera + MLX90614 (temp) + MAX30102 (HR/SpO2)
Hardware: Passive buzzer GPIO17, LED GPIO27, Touch sensor GPIO23
Dashboard: served over WiFi via Flask

GPS: Uses gpsd service or direct serial connection
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
from max30102 import MAX30102  # This will now find the local file
import serial
import pynmea2
import subprocess
import os

## ---------------------------------------------------------------------
# EMAIL CONFIG -- loaded from ~/email_config.py on the Pi
# ---------------------------------------------------------------------
import ssl

SMTP_HOST = "smtp.gmail.com"
SMTP_PORT = 465
SMTP_TIMEOUT = 20          # seconds, prevents a hung thread

EMAIL_ENABLED = False
try:
    from email_config import EMAIL_SENDER, EMAIL_PASSWORD, EMAIL_RECEIVER
    if all([EMAIL_SENDER, EMAIL_PASSWORD, EMAIL_RECEIVER]):
        EMAIL_ENABLED = True
        print(f"[EMAIL] Alerts enabled -> {EMAIL_RECEIVER}")
    else:
        print("[EMAIL] email_config.py has empty values -- alerts disabled.")
except ImportError:
    EMAIL_SENDER = EMAIL_PASSWORD = EMAIL_RECEIVER = None
    print("[EMAIL] email_config.py not found -- alerts disabled.")


# ---------------------------------------------------------------------
# EMAIL ALERT FUNCTION
# ---------------------------------------------------------------------
def send_email(subject, body, alert_type):
    if not EMAIL_ENABLED:
        return

    now = time.time()
    last = _email_cooldowns.get(alert_type, 0)
    if now - last < EMAIL_COOLDOWN:
        remaining = int(EMAIL_COOLDOWN - (now - last))
        print(f"[EMAIL] {alert_type} cooldown active, {remaining}s left")
        return

    # Claim the cooldown slot immediately so two threads firing the same
    # alert type at once cannot both get through.
    _email_cooldowns[alert_type] = now

    msg = MIMEText(body)
    msg["Subject"] = subject
    msg["From"] = EMAIL_SENDER
    msg["To"] = EMAIL_RECEIVER

    ctx = ssl.create_default_context()

    for attempt in (1, 2):
        try:
            t0 = time.time()
            with smtplib.SMTP_SSL(SMTP_HOST, SMTP_PORT,
                                  context=ctx, timeout=SMTP_TIMEOUT) as server:
                server.login(EMAIL_SENDER, EMAIL_PASSWORD)
                server.sendmail(EMAIL_SENDER, EMAIL_RECEIVER, msg.as_string())
            print(f"[EMAIL] {alert_type} alert sent in {time.time()-t0:.1f}s")
            return

        except smtplib.SMTPAuthenticationError:
            print("[EMAIL ERROR] Login rejected. Regenerate the Gmail "
                  "App Password in email_config.py.")
            return                      # retrying will not help

        except (smtplib.SMTPServerDisconnected, TimeoutError, OSError) as e:
            if attempt == 1:
                print(f"[EMAIL] Network issue ({type(e).__name__}), retrying...")
                time.sleep(2.0)
                continue
            print(f"[EMAIL ERROR] {alert_type} failed after retry: {e}")
            _email_cooldowns[alert_type] = 0    # allow the next alert to try

        except Exception as e:
            print(f"[EMAIL ERROR] {type(e).__name__}: {e}")
            _email_cooldowns[alert_type] = 0
            return

# ---------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------
MODEL_PATH        = "yolov8n.pt"
CONF              = 0.45
IOU               = 0.45
IMGSZ             = 256
CAM_W, CAM_H      = 480, 360
FREE_FALL_THRESH  = 4.5  #base is 3.0
IMPACT_THRESH     = 12.0 #change to 20 later 
IMPACT_WINDOW     = 1.5  #change to 1.5
SAMPLE_RATE       = 0.02
MAX_ALERTS        = 20
TEMP_ALERT_F      = 103.0
HR_ALERT_BPM      = 120
EMAIL_COOLDOWN    = 300
# --- Heart rate (MAX30102) ---
HR_FINGER_THRESHOLD = 50000   # IR below this means no skin contact
HR_LED_CURRENT      = 0x24    # raise toward 0x3F if IR reads low
HR_MIN_AMPLITUDE    = 40      # min AC swing before beats are trusted
HR_REFRACTORY       = 0.30    # seconds, blocks double-triggering
HR_MIN_BEATS        = 4       # intervals needed before publishing
HR_MAX_SPREAD       = 0.30    # interval spread ratio for "good" quality

# --- Dashboard URL used in email alerts ---
import socket
DASHBOARD_URL = f"http://{socket.gethostname()}.local:5000"
# --- GPS Configuration ---
GPS_SERIAL_PORT   = "/dev/serial0"
GPS_BAUDRATE      = 9600
GPS_USE_GPSD      = True
GPS_TIMEOUT       = 5.0

# GPIO pins
PIN_BUZZER  = 17
PIN_LED     = 27
PIN_TOUCH   = 23

# MLX90614 I2C
MLX_ADDR      = 0x5A
MLX_RAM_TOBJ1 = 0x07

# MAX30102 I2C
MAX_ADDR      = 0x57

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
    "gps_fix": False,
    "gps_satellites": 0,
    "gps_quality": "No Fix",
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
    "hr_quality": "no contact",
    "hr_contact": False,
    "detections": [],
    "fps": 0.0,
}

frame_lock  = threading.Lock()
latest_jpeg = None
stop_event  = threading.Event()

_email_cooldowns = {
    "fall": 0,
    "temp": 0,
    "hr": 0
}


# ---------------------------------------------------------------------
# GPS THREAD - Real NEO-6M
# ---------------------------------------------------------------------
class GPSReceiver:
    def __init__(self):
        self.serial = None
        self.gpsd_running = False
        self.fix = False
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.satellites = 0
        self.quality = "No Fix"
        
    def start_gpsd(self):
        try:
            result = subprocess.run(['pgrep', 'gpsd'], capture_output=True, text=True)
            if result.stdout.strip():
                print("[GPS] gpsd already running")
                self.gpsd_running = True
                return True
            
            print("[GPS] Starting gpsd...")
            subprocess.run(['sudo', 'gpsd', GPS_SERIAL_PORT, '-F', '/var/run/gpsd.sock'], 
                          check=True, capture_output=True)
            self.gpsd_running = True
            print("[GPS] gpsd started successfully")
            return True
        except Exception as e:
            print(f"[GPS] Failed to start gpsd: {e}")
            return False
    
    def connect_direct(self):
        try:
            self.serial = serial.Serial(GPS_SERIAL_PORT, GPS_BAUDRATE, timeout=1.0)
            print(f"[GPS] Connected directly to {GPS_SERIAL_PORT}")
            return True
        except Exception as e:
            print(f"[GPS] Failed to connect to {GPS_SERIAL_PORT}: {e}")
            return False
    
    def read_gpsd(self):
        try:
            import gps
            session = gps.gps(mode=gps.WATCH_ENABLE)
            
            start_time = time.time()
            while time.time() - start_time < GPS_TIMEOUT:
                try:
                    report = session.next()
                    if report['class'] == 'TPV':
                        if hasattr(report, 'lat') and hasattr(report, 'lon'):
                            self.lat = report.lat
                            self.lon = report.lon
                            self.fix = True
                            if hasattr(report, 'alt'):
                                self.alt = report.alt
                            return
                except Exception:
                    pass
                time.sleep(0.1)
                
        except Exception as e:
            print(f"[GPS] gpsd read error: {e}")
            self.fix = False
            
    def read_serial_nmea(self):
        try:
            start_time = time.time()
            while time.time() - start_time < GPS_TIMEOUT:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('ascii', errors='ignore')
                    if line.startswith('$GPGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            if msg.gps_qual > 0:
                                self.lat = msg.latitude
                                self.lon = msg.longitude
                                self.alt = msg.altitude
                                self.satellites = msg.num_sats
                                self.quality = ["No Fix", "GPS", "DGPS", "PPS", "RTK", "Float RTK", 
                                               "Estimated", "Manual", "Simulation"][msg.gps_qual]
                                self.fix = True
                                return
                        except pynmea2.ParseError:
                            pass
                time.sleep(0.1)
                
        except Exception as e:
            print(f"[GPS] Serial read error: {e}")
            self.fix = False
    
    def get_position(self):
        if GPS_USE_GPSD:
            self.read_gpsd()
        else:
            self.read_serial_nmea()
        
        if self.fix:
            return {
                "lat": round(self.lat, 6),
                "lon": round(self.lon, 6),
                "alt": round(self.alt, 1),
                "fix": True,
                "satellites": self.satellites,
                "quality": self.quality
            }
        else:
            return {
                "lat": "No Fix",
                "lon": "No Fix",
                "alt": "No Fix",
                "fix": False,
                "satellites": 0,
                "quality": "No Fix"
            }

def gps_thread():
    gps_receiver = GPSReceiver()
    
    if GPS_USE_GPSD:
        if not gps_receiver.start_gpsd():
            print("[GPS] Falling back to direct serial connection")
            gps_receiver.connect_direct()
    else:
        if not gps_receiver.connect_direct():
            print("[GPS] WARNING: Cannot connect to GPS module")
    
    print("[GPS] Waiting for GPS fix...")
    fix_timeout = time.time() + 60
    
    while not stop_event.is_set():
        try:
            position = gps_receiver.get_position()
            
            with state_lock:
                state["lat"] = position["lat"]
                state["lon"] = position["lon"]
                state["alt"] = position["alt"]
                state["gps_fix"] = position["fix"]
                state["gps_satellites"] = position.get("satellites", 0)
                state["gps_quality"] = position.get("quality", "No Fix")
            
            if position["fix"]:
                print(f"[GPS] Fix acquired: {position['lat']}, {position['lon']} "
                      f"(Satellites: {position.get('satellites', 0)})")
                fix_timeout = time.time() + 60
            elif time.time() > fix_timeout:
                if time.time() % 30 < 1:
                    print("[GPS] Still waiting for GPS fix... (Check antenna placement)")
            
            time.sleep(1.0)
            
        except Exception as e:
            print(f"[GPS ERROR] {e}")
            time.sleep(2.0)

# ---------------------------------------------------------------------
# MLX90614 TEMPERATURE THREAD
# ---------------------------------------------------------------------
def read_mlx90614(bus):
    try:
        raw    = bus.read_word_data(MLX_ADDR, MLX_RAM_TOBJ1)
        temp_k = raw * 0.02
        temp_c = temp_k - 273.15
        return temp_c
    except Exception:
        return None

def temp_thread():
    try:
        bus = smbus2.SMBus(1)
    except Exception as e:
        print(f"[TEMP ERROR] Could not open I2C bus: {e}")
        return

    while not stop_event.is_set():
        try:
            temp_c = read_mlx90614(bus)
            if temp_c is None:
                time.sleep(1.0)
                continue
                
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
                    print(f"[TEMP ALERT] {temp_f:.1f}F exceeds threshold!")
                    threading.Thread(target=send_email, daemon=True, args=(
                        "HIGH TEMP ALERT -- Worker Overheating",
                        f"Worker temperature exceeded 103F.\n\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')}\nTemp: {temp_f:.1f}F / {temp_c:.1f}C\nLat: {state['lat']}\nLon: {state['lon']}\n\nDashboard: http://192.168.1.212:5000",
                        "temp"
                    )).start()
                elif temp_f < (TEMP_ALERT_F - 2.0) and state["temp_alert"]:
                    state["temp_alert"] = False

        except Exception as e:
            print(f"[TEMP ERROR] {e}")

        time.sleep(1.0)

# ---------------------------------------------------------------------
# HEART RATE THREAD -- MAX30102
# ---------------------------------------------------------------------
# HEART RATE -- MAX30102 DRIVER
# ---------------------------------------------------------------------
class MAX30102Driver:
    """Minimal MAX30102 driver over smbus2. No external library needed."""
 
    REG_INTR_STATUS_1 = 0x00
    REG_INTR_STATUS_2 = 0x01
    REG_INTR_ENABLE_1 = 0x02
    REG_INTR_ENABLE_2 = 0x03
    REG_FIFO_WR_PTR   = 0x04
    REG_OVF_COUNTER   = 0x05
    REG_FIFO_RD_PTR   = 0x06
    REG_FIFO_DATA     = 0x07
    REG_FIFO_CONFIG   = 0x08
    REG_MODE_CONFIG   = 0x09
    REG_SPO2_CONFIG   = 0x0A
    REG_LED1_PA       = 0x0C
    REG_LED2_PA       = 0x0D
    REG_PILOT_PA      = 0x10
    REG_PART_ID       = 0xFF
 
    def __init__(self, bus_num=1, address=MAX_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
 
    def _w(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val)
 
    def _r(self, reg):
        return self.bus.read_byte_data(self.address, reg)
 
    def get_part_id(self):
        return self._r(self.REG_PART_ID)
 
    def setup(self):
        self._w(self.REG_MODE_CONFIG, 0x40)          # soft reset
        time.sleep(0.1)
        self._w(self.REG_INTR_ENABLE_1, 0xC0)
        self._w(self.REG_INTR_ENABLE_2, 0x00)
        self._w(self.REG_FIFO_WR_PTR, 0x00)
        self._w(self.REG_OVF_COUNTER, 0x00)
        self._w(self.REG_FIFO_RD_PTR, 0x00)
        self._w(self.REG_FIFO_CONFIG, 0x1F)          # no averaging, rollover on
        self._w(self.REG_MODE_CONFIG, 0x03)          # SpO2 mode: LED1 red, LED2 IR
        self._w(self.REG_SPO2_CONFIG, 0x27)          # 4096nA, 100 sps, 411us, 18 bit
        self._w(self.REG_LED1_PA, HR_LED_CURRENT)
        self._w(self.REG_LED2_PA, HR_LED_CURRENT)
        self._w(self.REG_PILOT_PA, 0x7F)
 
    def data_available(self):
        """Unread samples sitting in the FIFO, 0 to 31."""
        wr = self._r(self.REG_FIFO_WR_PTR)
        rd = self._r(self.REG_FIFO_RD_PTR)
        n = wr - rd
        return n + 32 if n < 0 else n
 
    def read_fifo(self):
        """One sample. Returns (red, ir) as 18 bit counts."""
        d = self.bus.read_i2c_block_data(self.address, self.REG_FIFO_DATA, 6)
        red = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x03FFFF
        ir  = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x03FFFF
        return red, ir
 
    def shutdown(self):
        try:
            self._w(self.REG_MODE_CONFIG, 0x80)
            self.bus.close()
        except Exception:
            pass
 
 
# ---------------------------------------------------------------------
# HEART RATE -- BEAT DETECTOR
# ---------------------------------------------------------------------
class BeatDetector:
    """
    Adaptive threshold peak detector for reflectance PPG.
 
    Feed it IR samples with feed(ir). Returns True on the sample where a
    beat is confirmed. Read .bpm and .quality for the current estimate.
    """
 
    SMOOTH_N   = 4       # moving average window
    DC_ALPHA   = 0.03    # DC tracker, about 0.3 s at 100 sps
    ENV_DECAY  = 0.995   # amplitude envelope decay
    TRIG_RATIO = 0.35    # fraction of envelope that counts as a beat
    HIST_LEN   = 8       # intervals kept for the median
 
    def __init__(self):
        self.reset()
 
    def reset(self):
        self._window = []
        self._dc = None
        self._env = 0.0
        self._prev_ac = 0.0
        self._last_beat = 0.0
        self._intervals = []
        self.bpm = 0
        self.quality = "no contact"
 
    def feed(self, ir, now):
        # 1. moving average
        self._window.append(ir)
        if len(self._window) > self.SMOOTH_N:
            self._window.pop(0)
        smooth = sum(self._window) / len(self._window)
 
        # 2. DC removal
        if self._dc is None:
            self._dc = smooth
            self._prev_ac = 0.0
            self._last_beat = now
            return False
        self._dc = self._dc * (1.0 - self.DC_ALPHA) + smooth * self.DC_ALPHA
        ac = smooth - self._dc
 
        # 3. amplitude envelope
        mag = abs(ac)
        if mag > self._env:
            self._env = mag
        else:
            self._env = self._env * self.ENV_DECAY + mag * (1.0 - self.ENV_DECAY)
 
        beat = False
 
        # 4. threshold crossing with refractory period
        if self._env >= HR_MIN_AMPLITUDE:
            trig = self._env * self.TRIG_RATIO
            if (self._prev_ac <= trig < ac
                    and (now - self._last_beat) > HR_REFRACTORY):
                delta = now - self._last_beat
                self._last_beat = now
                if 0.30 < delta < 2.0:          # 30 to 200 BPM
                    self._intervals.append(delta)
                    if len(self._intervals) > self.HIST_LEN:
                        self._intervals.pop(0)
                    beat = True
 
        self._prev_ac = ac
 
        # 5. estimate from the MEDIAN interval
        n = len(self._intervals)
        if n >= HR_MIN_BEATS:
            s = sorted(self._intervals)
            med = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0
            spread = (s[-1] - s[0]) / med if med > 0 else 9.9
            self.bpm = int(round(60.0 / med))
            self.quality = "good" if spread <= HR_MAX_SPREAD else "noisy"
        else:
            self.bpm = 0
            self.quality = f"settling {n}/{HR_MIN_BEATS}"
 
        return beat
 
 
# ---------------------------------------------------------------------
# HEART RATE THREAD -- MAX30102
# ---------------------------------------------------------------------
def hr_thread():
    sensor = None
    try:
        sensor = MAX30102Driver()
        part_id = sensor.get_part_id()
 
        if part_id == 0x15:
            print(f"[HR] MAX30102 confirmed (Part ID: 0x{part_id:02X})")
        elif part_id == 0x11:
            print(f"[HR] MAX30100 detected (Part ID: 0x{part_id:02X}) -- unsupported")
            sensor.shutdown()
            return
        else:
            print(f"[HR] Unknown sensor (Part ID: 0x{part_id:02X})")
            sensor.shutdown()
            return
 
        sensor.setup()
        print("[HR] MAX30102 ready on I2C. Place fingertip on the sensor.")
 
    except Exception as e:
        print(f"[HR ERROR] Could not init MAX30102: {e}")
        if sensor:
            sensor.shutdown()
        return
 
    detector = BeatDetector()
    contact = False
    last_status = 0.0
    consecutive_errors = 0
 
    try:
        while not stop_event.is_set():
            try:
                pending = sensor.data_available()
 
                if pending == 0:
                    time.sleep(0.005)
                    continue
 
                # Drain the FIFO. Only the newest sample drives detection if
                # we fell behind, but every sample is consumed so pointers
                # stay in sync.
                for _ in range(min(pending, 16)):
                    red, ir = sensor.read_fifo()
                    now = time.time()
 
                    # ---- contact check ----
                    if ir < HR_FINGER_THRESHOLD:
                        if contact:
                            print("[HR] Contact lost.")
                            contact = False
                            detector.reset()
                            with state_lock:
                                state["hr_bpm"] = 0
                                state["hr_contact"] = False
                                state["hr_quality"] = "no contact"
                        continue
 
                    if not contact:
                        contact = True
                        detector.reset()
                        print(f"[HR] Contact detected (IR={ir}).")
                        with state_lock:
                            state["hr_contact"] = True
 
                    detector.feed(ir, now)
 
                consecutive_errors = 0
 
                # ---- publish + alerting ----
                if contact:
                    bpm = detector.bpm
                    quality = detector.quality
 
                    with state_lock:
                        state["hr_bpm"] = bpm
                        state["hr_quality"] = quality
 
                        # Alert only on trustworthy data. A loose finger
                        # otherwise produces false tachycardia emails.
                        if (quality == "good"
                                and bpm >= HR_ALERT_BPM
                                and not state["hr_alert"]):
                            state["hr_alert"] = True
                            alert = {
                                "time": time.strftime("%H:%M:%S"),
                                "bpm":  bpm,
                                "lat":  state["lat"],
                                "lon":  state["lon"],
                            }
                            state["hr_alerts"].insert(0, alert)
                            if len(state["hr_alerts"]) > MAX_ALERTS:
                                state["hr_alerts"].pop()
                            print(f"[HR ALERT] {bpm} BPM exceeds threshold!")
                            threading.Thread(target=send_email, daemon=True, args=(
                                "HIGH HEART RATE ALERT -- Tachycardia Detected",
                                f"Worker heart rate exceeded {HR_ALERT_BPM} BPM.\n\n"
                                f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                                f"BPM: {bpm}\n"
                                f"Lat: {state['lat']}\n"
                                f"Lon: {state['lon']}\n\n"
                                f"Dashboard: {DASHBOARD_URL}",
                                "hr"
                            )).start()
 
                        elif (quality == "good"
                                and bpm < (HR_ALERT_BPM - 10)
                                and state["hr_alert"]):
                            state["hr_alert"] = False
 
                    if time.time() - last_status > 2.0:
                        print(f"[HR] {bpm} BPM  ({quality})")
                        last_status = time.time()
 
            except OSError as e:
                # I2C hiccup. Back off briefly, re-init after repeated failures.
                consecutive_errors += 1
                print(f"[HR ERROR] I2C: {e} (x{consecutive_errors})")
                time.sleep(0.2)
                if consecutive_errors >= 10:
                    print("[HR] Re-initializing sensor.")
                    try:
                        sensor.setup()
                        detector.reset()
                        contact = False
                        consecutive_errors = 0
                    except Exception as re_e:
                        print(f"[HR ERROR] Re-init failed: {re_e}")
                        time.sleep(2.0)
 
            except Exception as e:
                print(f"[HR ERROR] {e}")
                time.sleep(0.1)
 
    finally:
        sensor.shutdown()
        print("[HR] Sensor stopped.")

# ---------------------------------------------------------------------
# BUZZER + LED THREAD
# ---------------------------------------------------------------------
def alert_hardware_thread():
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
            GPIO.output(PIN_LED, GPIO.LOW)
            GPIO.output(PIN_BUZZER, GPIO.LOW)
            time.sleep(0.1)

# ---------------------------------------------------------------------
# TOUCH SENSOR THREAD
# ---------------------------------------------------------------------
def touch_thread():
    debounce_counter = 0
    last_reading = False
    
    while not stop_event.is_set():
        reading = GPIO.input(PIN_TOUCH)
        
        if reading == last_reading:
            debounce_counter += 1
            if debounce_counter >= 5:
                with state_lock:
                    state["helmet_on"] = bool(reading)
        else:
            debounce_counter = 0
            last_reading = reading
            
        time.sleep(0.02)

# ---------------------------------------------------------------------
# IMU + FALL DETECTION THREAD
# ---------------------------------------------------------------------
def imu_thread():
    try:
        sensor = mpu6050(0x68)
    except Exception as e:
        print(f"[IMU ERROR] Could not init MPU6050: {e}")
        return
        
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
                        if not state["fall_active"]:
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
                            threading.Thread(target=send_email, daemon=True, args=(
                                "FALL DETECTED -- Worker Down",
                                f"A fall was detected.\n\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')}\nSMV: {round(smv, 2)} m/s^2\nLat: {state['lat']}\nLon: {state['lon']}\n\nDashboard: http://192.168.1.212:5000",
                                "fall"
                            )).start()
                        
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
# CAMERA + YOLO THREAD
# ---------------------------------------------------------------------
def camera_thread():
    global latest_jpeg

    try:
        cam = Picamera2()
        cfg = cam.create_preview_configuration(
            main={"size": (CAM_W, CAM_H), "format": "RGB888"},
            controls={"FrameRate": 30}
        )
        cam.configure(cfg)
        cam.start()
        time.sleep(0.8)
        print(f"[CAMERA] Ready ({CAM_W}x{CAM_H})")
    except Exception as e:
        print(f"[CAMERA ERROR] Could not init camera: {e}")
        return

    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"[YOLO ERROR] Could not load model: {e}")
        return
        
    font  = cv2.FONT_HERSHEY_SIMPLEX
    fps   = 0.0
    fc    = 0
    t_ref = time.time()

    try:
        while not stop_event.is_set():
            try:
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
                    elapsed = time.time() - t_ref
                    if elapsed > 0:
                        fps = 10.0 / elapsed
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
                            
            except Exception as e:
                print(f"[CAMERA ERROR] Frame capture error: {e}")
                time.sleep(0.1)
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
    <div class="panel-label">// GPS &mdash; NEO-6M</div>
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
    <div class="panel-label">// HEART RATE &mdash; MAX30102</div>
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

# ---------------------------------------------------------------------
# FLASK ROUTES
# ---------------------------------------------------------------------
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

@app.route("/api/acknowledge_hr", methods=["POST"])
def acknowledge_hr():
    with state_lock:
        state["hr_alert"] = False
    print("[ACK] HR alert acknowledged.")
    return jsonify({"status": "cleared"})

def mjpeg_generator():
    global latest_jpeg
    while True:
        with frame_lock:
            frame = latest_jpeg
        if frame:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)

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
        threading.Thread(target=gps_thread,            daemon=True, name="GPS"),
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
