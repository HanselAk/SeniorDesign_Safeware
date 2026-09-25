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

4. Open Firefox on the Jetson at http://127.0.0.1:5000 . The helmet camera
   panel should show video with YOLO boxes; detections, FPS and PPE fields
   should update. Press Ctrl+C in Terminal 2 to stop the camera; within four
   seconds the dashboard should show CAMERA OFFLINE and UNVERIFIED.

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

TROUBLESHOOTING
If Terminal 2 says "No module named ultralytics" or "No module named cv2",
check that its prompt starts with (ppe_env) and that you're on the Jetson.
If the page says "camera offline", look for an error in Terminal 2 and first
verify ~/Documents/PPE_Safety_Dashboard_Live_Camera/best.pt exists.
If port 5000 is already in use, stop the old dashboard in its terminal with
Ctrl+C before starting the new copy.
