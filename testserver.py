from flask import Flask, request, jsonify
from datetime import datetime

app = Flask(__name__)

# Optional: log to a file as well as console
LOG_FILE = "mpu9250_log.csv"

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.get_json(silent=True)
    
    if not data:
        print("Received empty or invalid JSON")
        return jsonify({"status": "error", "message": "Invalid JSON"}), 400

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # Pretty console output
    print(f"[{timestamp}]")
    print(f"  Accel (g):   x={data.get('ax'):>8}  y={data.get('ay'):>8}  z={data.get('az'):>8}")
    print(f"  Gyro (°/s):  x={data.get('gx'):>8}  y={data.get('gy'):>8}  z={data.get('gz'):>8}")
    print(f"  Mag (µT):    x={data.get('mx'):>8}  y={data.get('my'):>8}  z={data.get('mz'):>8}")
    print(f"  Temp (°C):   {data.get('temp')}")
    print("-" * 50)

    # Append to CSV for later analysis
    try:
        with open(LOG_FILE, "a") as f:
            f.write(f"{timestamp},{data.get('ax')},{data.get('ay')},{data.get('az')},"
                    f"{data.get('gx')},{data.get('gy')},{data.get('gz')},"
                    f"{data.get('mx')},{data.get('my')},{data.get('mz')},"
                    f"{data.get('temp')}\n")
    except Exception as e:
        print(f"CSV write error: {e}")

    return jsonify({"status": "ok"}), 200


@app.route('/', methods=['GET'])
def index():
    return "MPU9250 receiver is running. POST data to /data"


if __name__ == '__main__':
    # Create CSV header if file doesn't exist
    try:
        open(LOG_FILE, "x").write(
            "timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,temp\n"
        )
    except FileExistsError:
        pass

    print("Starting MPU9250 server on 0.0.0.0:5000")
    print("Waiting for ESP32 data...\n")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
