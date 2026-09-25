"""Send IMX219 CSI frames and YOLO results to the local dashboard.

Run in the same Python environment as the dashboard. Uses GStreamer to capture
the camera because pip's OpenCV build commonly lacks GStreamer support.
"""

import base64
import argparse
import subprocess
import sys
import time
import urllib.error
import urllib.request
import json

import cv2
import numpy as np
from ultralytics import YOLO


URL = "http://127.0.0.1:5000/api/camera/update"


def camera_frames(process):
    pending = bytearray()
    while True:
        chunk = process.stdout.read(65536)
        if not chunk:
            break
        pending.extend(chunk)
        while True:
            start = pending.find(b"\xff\xd8")
            if start < 0:
                pending.clear()
                break
            if start:
                del pending[:start]
            end = pending.find(b"\xff\xd9", 2)
            if end < 0:
                if len(pending) > 5_000_000:
                    pending.clear()
                break
            yield bytes(pending[:end + 2])
            del pending[:end + 2]


def main():
    parser = argparse.ArgumentParser(description="Stream Jetson CSI camera detections to the PPE dashboard")
    parser.add_argument("--fast", action="store_true", help="Use 640x360 camera frames and 416px inference for higher FPS")
    args = parser.parse_args()
    model = YOLO("best.pt")
    command = [
        "gst-launch-1.0", "-q", "nvarguscamerasrc", "sensor-id=0", "!",
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1", "!",
    ]
    if args.fast:
        command += ["nvvidconv", "!", "video/x-raw(memory:NVMM),width=640,height=360,format=NV12", "!"]
    command += ["nvjpegenc", "!", "fdsink", "fd=1", "sync=false"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=0)
    print("Camera running. Open http://127.0.0.1:5000 in the browser. Press Ctrl+C here to stop.", flush=True)
    last_sent = 0.0
    try:
        for jpeg in camera_frames(process):
            # Skip intermediate camera frames to keep the view responsive.
            if time.monotonic() - last_sent < (0.08 if args.fast else 0.12):
                continue
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            result = model.predict(frame, device=0, imgsz=416 if args.fast else 640,
                                   conf=0.5, verbose=False)[0]
            annotated = result.plot()
            ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if not ok:
                continue
            now = time.monotonic()
            fps = 1.0 / (now - last_sent) if last_sent else 0.0
            last_sent = now
            payload = {
                "frame_jpeg": base64.b64encode(encoded.tobytes()).decode("ascii"),
                "fps": fps,
                "detections": [
                    {"label": result.names[int(box.cls[0])], "confidence": float(box.conf[0])}
                    for box in result.boxes
                ],
            }
            request = urllib.request.Request(
                URL, data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=3):
                    pass
            except (urllib.error.URLError, TimeoutError) as exc:
                print(f"Dashboard unavailable ({exc}). Start demo_dashboard.py in another terminal.", flush=True)
        print(f"Camera stream ended (GStreamer exit code: {process.poll()}).", file=sys.stderr)
    except KeyboardInterrupt:
        print("Stopping camera.")
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
