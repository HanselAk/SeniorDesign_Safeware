JETSON ORIN NANO — LIVE IMX219 + YOLO DASHBOARD

This is a separate copy of the existing Flask dashboard. It receives live
detections from camera_to_dashboard.py. Keep your original folder as a backup.
Run commands on the Jetson, not on the Windows laptop.

1. Copy this entire folder to ~/Documents/ on the Jetson. Put your trusted
   trained best.pt file in this folder, beside camera_to_dashboard.py.
   The model file is not included in this download. If best.pt is already in
   your old dashboard folder, copy it into this new folder using Files.

2. Open Terminal 1:

   source ~/ppe_env/bin/activate
   cd ~/Documents/PPE_Safety_Dashboard_Live_Camera
   python demo_dashboard.py

3. Open Terminal 2:

   source ~/ppe_env/bin/activate
   cd ~/Documents/PPE_Safety_Dashboard_Live_Camera
   python camera_to_dashboard.py

   For higher speed with a smaller image, instead run:

   python camera_to_dashboard.py --fast

   The fast option downscales to 640x360 before JPEG transfer and runs YOLO
   with a 416-pixel input. Small or distant PPE may become harder to detect.
   To return to the original resolution, stop with Ctrl+C and run the command
   without --fast.

4. Open Firefox on the Jetson at http://127.0.0.1:5000 . The helmet camera
   panel should show video with YOLO boxes; detections, FPS and PPE fields
   should update. Press Ctrl+C in Terminal 2 to stop the camera; within four
   seconds the dashboard should show CAMERA OFFLINE and UNVERIFIED.

The live image refreshes independently of the dashboard metrics. The camera
worker attempts up to about eight model updates per second; the FPS counter
shows its actual achieved speed, which depends on the Jetson and scene.

Uses the existing Flask, OpenCV, PyTorch and Ultralytics in your ppe_env.
The camera process uses gst-launch-1.0 and nvarguscamerasrc sensor-id=0, which
you already tested successfully. No new packages should be needed.

INTERPRETING RESULTS
The camera fields are real model outputs; heart rate, temperature, movement,
and location remain simulated. UNKNOWN means PPE could not be verified in that
frame or the camera is offline. MISSING requires a person plus an explicit
"no helmet" or "no vest" model detection at 0.5 confidence or higher.
The model's labels are not tied to individual people: with multiple people,
the dashboard cannot judge each person's compliance. This is a prototype,
not a safety system. Alerts are added when an explicit missing-PPE detection
appears; previously logged events remain in the recent alerts list.

ESP32 MPU6050 FALL DETECTOR (separate third terminal)
The camera and dashboard keep running as before. Wearable heart rate,
temperature and location remain demo data. The movement card changes to
"LIVE MPU6050" once the Jetson receives ESP32 readings over USB.

Generic ESP32 -> MPU6050 breakout wiring (power disconnected while wiring):
  ESP32 3V3 -> VCC     ESP32 GND -> GND
  ESP32 GPIO21 -> SDA  ESP32 GPIO22 -> SCL
  MPU6050 AD0 -> GND (address 0x68)
If your ESP32 board uses different I2C pins, change Wire.begin(21, 22, 100000)
in esp32_mpu6050/esp32_mpu6050.ino. Use 3.3V on the sensor breakout; check the
particular breakout's markings. Upload the sketch with Arduino IDE; choose
your ESP32 board and Serial Monitor baud rate 115200. You should see lines
containing ax, ay, az. At rest SMV should be about 9.8 m/s^2. Close Serial
Monitor before starting the Jetson bridge, since only one app can own the port.

On the Jetson, connect the ESP32's USB cable and open a third terminal:
  source ~/ppe_env/bin/activate
  cd ~/Documents/PPE_Safety_Dashboard_Live_Camera
  python -m pip install pyserial
  python fall_to_dashboard.py --demo

The demo command generates a test free-fall+impact sequence, writes
fall_log.csv, and sends a test fall alert to the running dashboard. Click
"Acknowledge active alerts" to clear its active warning. After confirming
the demo works, run the real sensor reader:
  python fall_to_dashboard.py

If the script reports multiple serial devices, identify the ESP32's port
with: ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
Then run: python fall_to_dashboard.py --port /dev/ttyUSB0
or replace the port with /dev/ttyACM0, as appropriate.
If access is denied, your user may need serial-port group access; do not
run the entire dashboard as root. The ESP32 sketch requires ±8g accel range
so 20 m/s^2 impact readings do not saturate at the sensor's default ±2g.
The detector watches for two consecutive low samples (<3 m/s^2), then an
impact (>=20 m/s^2) within 1.5 seconds; it suppresses repeat events for
three seconds and logs one normal sample in ten plus every event. These are
prototype thresholds: actual falls can be missed and ordinary movements can
trigger alerts. Use the generated demo sequence for initial testing. Do not
test by falling or dropping a person or electronic equipment.

TROUBLESHOOTING
If Terminal 2 says "No module named ultralytics" or "No module named cv2",
check that its prompt starts with (ppe_env) and that you're on the Jetson.
If the page says "camera offline", look for an error in Terminal 2 and first
verify ~/Documents/PPE_Safety_Dashboard_Live_Camera/best.pt exists.
If port 5000 is already in use, stop the old dashboard in its terminal with
Ctrl+C before starting the new copy.
