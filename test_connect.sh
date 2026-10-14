#!/usr/bin/env bash
set -euo pipefail

# MAVLink connection settings can be overridden, for example:
#   MAVLINK_PORT=/dev/ttyUSB0 MAVLINK_BAUD=115200 ./test_connect.sh
PORT="${MAVLINK_PORT:-/dev/ttyAMA0}"
BAUD="${MAVLINK_BAUD:-115200}"
TIMEOUT="${MAVLINK_TIMEOUT:-6}"

if [[ ! -e "$PORT" ]]; then
    echo "[FAIL] Không tìm thấy cổng MAVLink: $PORT"
    exit 1
fi

if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
    echo "[FAIL] Chưa cài pymavlink. Cài bằng: python3 -m pip install pymavlink"
    exit 1
fi

exec python3 - "$PORT" "$BAUD" "$TIMEOUT" <<'PY'
import sys
import time

from pymavlink import mavutil


port = sys.argv[1]
baud = int(sys.argv[2])
timeout = float(sys.argv[3])

print(f"MAVLink: {port} @ {baud} baud")
print("Chế độ kiểm tra: chỉ đọc telemetry; không arm và không điều khiển motor.")

try:
    mav = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    heartbeat = mav.wait_heartbeat(timeout=timeout)
except Exception as exc:
    print(f"[FAIL] Không mở được MAVLink: {exc}")
    raise SystemExit(1)

if heartbeat is None:
    print(f"[FAIL] Không nhận được heartbeat trong {timeout:.0f}s")
    raise SystemExit(1)

system_id = heartbeat.get_srcSystem()
component_id = heartbeat.get_srcComponent()
armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

print(
    f"[OK] Heartbeat: SYSID={system_id} COMPID={component_id} "
    f"type={heartbeat.type} autopilot={heartbeat.autopilot} "
    f"armed={'YES' if armed else 'NO'}"
)

# Ask the flight controller for normal telemetry. This changes stream rates only;
# it does not arm, move servos, or start motors.
mav.mav.request_data_stream_send(
    system_id,
    component_id,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,
    1,
)

latest = {}
counts = {}
deadline = time.monotonic() + timeout
while time.monotonic() < deadline:
    message = mav.recv_match(blocking=False)
    if message is None:
        time.sleep(0.01)
        continue
    name = message.get_type()
    if name == "BAD_DATA":
        continue
    latest[name] = message
    counts[name] = counts.get(name, 0) + 1

status = latest.get("SYS_STATUS")
fatal = False

if status is None:
    print("[FAIL] Không nhận được SYS_STATUS")
    fatal = True
else:
    present = int(status.onboard_control_sensors_present)
    enabled = int(status.onboard_control_sensors_enabled)
    health = int(status.onboard_control_sensors_health)
    print(
        f"SYS_STATUS: present=0x{present:08x} "
        f"enabled=0x{enabled:08x} health=0x{health:08x}"
    )

    sensors = [
        (0, "Gyro 3D", True),
        (1, "Accelerometer 3D", True),
        (2, "Compass/Magnetometer 3D", True),
        (3, "Barometer", True),
        (5, "GPS", False),
        (6, "Optical flow", False),
        (7, "Vision position", False),
        (8, "Laser position", False),
        (15, "Motor outputs", True),
        (16, "RC receiver", False),
        (20, "AHRS", False),
        (21, "Terrain", False),
        (24, "Logging", False),
        (25, "Battery", False),
        (26, "Proximity", False),
        (28, "Pre-arm check", False),
        (30, "Propulsion", False),
    ]

    for bit, label, required in sensors:
        mask = 1 << bit
        if not (present & mask):
            print(f"[INFO] {label}: không được báo là có mặt")
        elif not (enabled & mask):
            print(f"[INFO] {label}: có mặt nhưng đang disabled")
        elif health & mask:
            print(f"[ OK ] {label}: HEALTHY")
        else:
            print(f"[WARN] {label}: UNHEALTHY")
            if required:
                fatal = True

    voltage = int(getattr(status, "voltage_battery", 0))
    current = int(getattr(status, "current_battery", -1))
    remaining = int(getattr(status, "battery_remaining", -1))
    if voltage > 0:
        current_text = f"{current / 100.0:.2f}A" if current >= 0 else "unknown"
        remaining_text = f"{remaining}%" if remaining >= 0 else "unknown"
        print(f"[ OK ] Battery telemetry: {voltage / 1000.0:.2f}V, {current_text}, còn {remaining_text}")
    else:
        print("[WARN] Battery telemetry: chưa có điện áp pin")

attitude = latest.get("ATTITUDE")
if attitude is not None:
    print(
        f"[ OK ] Attitude: roll={attitude.roll:.3f} "
        f"pitch={attitude.pitch:.3f} yaw={attitude.yaw:.3f} rad"
    )
else:
    print("[WARN] Chưa nhận được ATTITUDE")

imu = latest.get("RAW_IMU") or latest.get("SCALED_IMU")
if imu is not None:
    values = []
    for field in ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag"):
        if hasattr(imu, field):
            values.append(f"{field}={getattr(imu, field)}")
    print("[ OK ] IMU data: " + " ".join(values))
else:
    print("[WARN] Chưa nhận được RAW_IMU/SCALED_IMU")

gps = latest.get("GPS_RAW_INT")
if gps is None:
    print("[WARN] Chưa nhận được GPS_RAW_INT")
else:
    fix_type = int(gps.fix_type)
    satellites = int(gps.satellites_visible)
    if fix_type >= 3 and satellites > 0:
        print(f"[ OK ] GPS: fix_type={fix_type}, satellites={satellites}")
    else:
        print(f"[WARN] GPS: chưa có fix, fix_type={fix_type}, satellites={satellites}")

servo = latest.get("SERVO_OUTPUT_RAW")
if servo is None:
    print("[WARN] Chưa nhận được SERVO_OUTPUT_RAW")
else:
    outputs = []
    for channel in range(1, 9):
        field = f"servo{channel}_raw"
        if hasattr(servo, field):
            outputs.append(f"S{channel}={getattr(servo, field)}")
    print("[ OK ] Servo/motor outputs: " + " ".join(outputs))
    if armed:
        print("[WARN] Pixhawk đang ARMED; không tiếp tục kiểm tra motor tự động.")
    else:
        print("[ OK ] Pixhawk đang DISARMED; script không làm motor quay.")

esc = latest.get("ESC_STATUS") or latest.get("ESC_TELEMETRY_1_TO_4")
if esc is None:
    print("[INFO] ESC telemetry: chưa nhận được; chưa thể xác nhận kết nối vật lý tới ESC/motor.")
else:
    print("[ OK ] ESC telemetry: đã nhận được")

print(f"Telemetry messages: {sum(counts.values())} trong {timeout:.0f}s")
if fatal:
    print("KẾT QUẢ: FAIL — có sensor bắt buộc unhealthy hoặc thiếu SYS_STATUS.")
    raise SystemExit(1)

print("KẾT QUẢ: PASS — MAVLink và các sensor chính đang trả telemetry.")
PY
