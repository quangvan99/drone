#!/usr/bin/env bash
set -euo pipefail

PORT="${MAVLINK_PORT:-/dev/ttyAMA0}"
BAUD="${MAVLINK_BAUD:-115200}"
PWM="${MOTOR_TEST_PWM:-1100}"
DURATION="${MOTOR_TEST_SECONDS:-2}"

if [[ ! -e "$PORT" ]]; then
    echo "[FAIL] Không tìm thấy cổng MAVLink: $PORT"
    exit 1
fi

if ! [[ "$PWM" =~ ^[0-9]+$ ]] || (( PWM < 1050 || PWM > 1200 )); then
    echo "[FAIL] MOTOR_TEST_PWM phải nằm trong khoảng 1050..1200 (hiện tại: $PWM)"
    exit 1
fi

if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[FAIL] MOTOR_TEST_SECONDS phải là số lớn hơn 0 và không quá 5 giây"
    exit 1
fi
if ! awk "BEGIN {exit !($DURATION > 0 && $DURATION <= 5)}"; then
    echo "[FAIL] MOTOR_TEST_SECONDS phải lớn hơn 0 và không quá 5 giây"
    exit 1
fi

echo "CẢNH BÁO: Motor sẽ quay lần lượt ở PWM ${PWM} trong ${DURATION}s mỗi motor."
echo "BẮT BUỘC tháo toàn bộ cánh quạt trước khi chạy test."

exec python3 - "$PORT" "$BAUD" "$PWM" "$DURATION" <<'PY'
import sys
import time

from pymavlink import mavutil


PORT = sys.argv[1]
BAUD = int(sys.argv[2])
PWM = int(sys.argv[3])
DURATION = float(sys.argv[4])


def fail(message):
    print(f"[FAIL] {message}")
    raise SystemExit(1)


def print_status_text(message):
    text = getattr(message, "text", "").strip()
    if text:
        print(f"[Pixhawk] {text}")


def wait_for_ack(mav, command, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = mav.recv_match(blocking=False)
        if message is None:
            time.sleep(0.01)
            continue
        if message.get_type() == "STATUSTEXT":
            print_status_text(message)
        elif message.get_type() == "COMMAND_ACK" and int(message.command) == command:
            return message
    return None


def stop_motor_test(mav, system_id, component_id):
    # A zero-throttle, zero-time motor-test command makes ArduPilot finish the
    # current motor test and return the outputs to their safe minimum.
    mav.mav.command_long_send(
        system_id,
        component_id,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        1,
        1,
        0,
        0,
        1,
        0,
        0,
    )


try:
    mav = mavutil.mavlink_connection(PORT, baud=BAUD, source_system=255)
except Exception as exc:
    fail(f"Không mở được MAVLink: {exc}")

heartbeat = mav.wait_heartbeat(timeout=8)
if heartbeat is None:
    fail("Không nhận được heartbeat từ Pixhawk")

system_id = heartbeat.get_srcSystem()
component_id = heartbeat.get_srcComponent()
print(
    f"[OK] Heartbeat SYSID={system_id} COMPID={component_id} "
    f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
)

if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    fail("Firmware không phải ArduPilot")
if heartbeat.type not in (
    mavutil.mavlink.MAV_TYPE_QUADROTOR,
    mavutil.mavlink.MAV_TYPE_HELICOPTER,
    mavutil.mavlink.MAV_TYPE_HEXAROTOR,
    mavutil.mavlink.MAV_TYPE_OCTOROTOR,
):
    fail(f"Loại phương tiện không phải multicopter: type={heartbeat.type}")
if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    fail("Pixhawk đang ARMED; không chạy motor-test để tránh chồng lệnh")

# Ask for output telemetry. This does not require GPS and does not arm the
# flight controller through the normal ARM command.
mav.mav.request_data_stream_send(
    system_id,
    component_id,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,
    1,
)

motor_command = mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST
accepted = mavutil.mavlink.MAV_RESULT_ACCEPTED
in_progress = mavutil.mavlink.MAV_RESULT_IN_PROGRESS
tested = []

try:
    for motor in range(1, 5):
        print(f"Motor {motor}: gửi test ở {PWM} PWM trong {DURATION:.1f}s...")
        mav.mav.command_long_send(
            system_id,
            component_id,
            motor_command,
            0,
            motor,
            1,          # throttle type: direct PWM
            PWM,
            DURATION,
            1,          # one motor in this test command
            0,
            0,
        )

        ack = wait_for_ack(mav, motor_command, max(3.0, DURATION + 2.0))
        if ack is None:
            fail(f"Motor {motor}: không nhận được COMMAND_ACK")

        result = int(ack.result)
        if result not in (accepted, in_progress):
            fail(f"Motor {motor}: lệnh bị từ chối (MAV_RESULT={result})")

        print(f"[OK] Motor {motor}: COMMAND_ACK={result}; đang chờ hoàn tất...")
        deadline = time.monotonic() + DURATION
        while time.monotonic() < deadline:
            message = mav.recv_match(blocking=False)
            if message is not None and message.get_type() == "STATUSTEXT":
                print_status_text(message)
            time.sleep(0.01)
        tested.append(motor)
finally:
    try:
        stop_motor_test(mav, system_id, component_id)
        print("[OK] Đã gửi lệnh dừng motor-test và đưa output về mức an toàn.")
    except Exception as exc:
        print(f"[WARN] Không gửi được lệnh dừng motor-test: {exc}")

print(f"KẾT QUẢ: PASS — đã test xong motor {', '.join(map(str, tested))}.")
PY
