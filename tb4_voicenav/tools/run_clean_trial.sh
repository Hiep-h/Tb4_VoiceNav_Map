#!/bin/bash
set -uo pipefail

NAME="$1"
GOAL_X="$2"
GOAL_Y="$3"
OUT_PREFIX="$4"
LOG_DIR="$5"

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
TOOLS_DIR="$REPO/tb4_voicenav/tools"
MAP_YAML="$REPO/tb4_voicenav/maps/map.yaml"

mkdir -p "$LOG_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u

echo "=== [1/6] stop_all.sh (don sach truoc trial $NAME) ==="
bash "$REPO/scripts/stop_all.sh" 2>&1 | tee "$LOG_DIR/stop_all_before.log"

echo "=== [2/6] Khoi dong headless_nav_eval.launch.py ==="
cd "$TOOLS_DIR"
ros2 launch headless_nav_eval.launch.py \
    map:="$MAP_YAML" software_render:=true x:=1.0 y:=1.0 yaw:=0.0 \
    > "$LOG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launch PID: $LAUNCH_PID"

echo "=== [3/6] Ghim CPU (pin_cpus.sh, chay song song khong cho) ==="
bash "$REPO/scripts/pin_cpus.sh" > "$LOG_DIR/pin_cpus.log" 2>&1 &
PIN_PID=$!

echo "=== [4/6] Cho bt_navigator ACTIVE (toi da 400s) ==="
READY=0
for i in $(seq 1 80); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "LOI: tien trinh launch da chet truoc khi san sang"
        break
    fi
    if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -qi active; then
        READY=1
        echo "bt_navigator ACTIVE sau khoang $((i * 5))s"
        break
    fi
    sleep 5
done

if [ "$READY" -ne 1 ]; then
    echo "LOI: bt_navigator khong active sau 400s, huy trial $NAME"
    kill -INT "$LAUNCH_PID" 2>/dev/null
    wait "$PIN_PID" 2>/dev/null
    bash "$REPO/scripts/stop_all.sh" 2>&1 | tee "$LOG_DIR/stop_all_after_fail.log"
    exit 1
fi

python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
import sys
import time

rclpy.init()
node = Node('tf_check', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
buf = Buffer()
TransformListener(buf, node)
ok = False
t0 = time.time()
while time.time() - t0 < 120:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        ok = True
        break
    except Exception:
        pass
print('TF map->base_link OK' if ok else 'TF map->base_link LOI sau 120s')
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PYEOF
TF_RC=$?
if [ "$TF_RC" -ne 0 ]; then
    echo "LOI HA TANG: TF map->base_link khong san sang sau 120s (AMCL co the chua active do loi lifecycle_manager). Huy trial $NAME, KHONG chay run_trial.py."
    kill -INT "$LAUNCH_PID" 2>/dev/null
    wait "$PIN_PID" 2>/dev/null
    bash "$REPO/scripts/stop_all.sh" 2>&1 | tee "$LOG_DIR/stop_all_after_tf_fail.log"
    exit 2
fi
set -e

echo "=== [5/6] Chay run_trial.py cho goal $NAME ($GOAL_X, $GOAL_Y) ==="
mkdir -p "$(dirname "$OUT_PREFIX")"
timeout 900 python3 "$TOOLS_DIR/run_trial.py" \
    --name "$NAME" --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --out-prefix "$OUT_PREFIX" \
    2>&1 | tee "$LOG_DIR/run_trial.log"
TRIAL_RC=${PIPESTATUS[0]}
echo "run_trial.py exit code: $TRIAL_RC"

echo "=== [6/6] Don sach sau trial $NAME ==="
kill -INT "$LAUNCH_PID" 2>/dev/null
wait "$PIN_PID" 2>/dev/null
bash "$REPO/scripts/stop_all.sh" 2>&1 | tee "$LOG_DIR/stop_all_after.log"

echo "Trial $NAME xong. Log tai $LOG_DIR, ket qua tai ${OUT_PREFIX}_result.json"
exit "$TRIAL_RC"
