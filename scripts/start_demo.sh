#!/bin/bash
set -u
cd "$(dirname "$0")/.."
set +u
source /opt/ros/humble/setup.bash
set -u
LOG="$HOME/tb4_voicenav_data/launch.log"
mkdir -p "$(dirname "$LOG")"
MAX_TRIES=${MAX_TRIES:-3}
ODOM_WAIT_S=${ODOM_WAIT_S:-300}

cleanup() {
    bash scripts/stop_all.sh >/dev/null 2>&1
}
trap 'cleanup; exit 0' INT TERM

for try in $(seq 1 "$MAX_TRIES"); do
    echo "=== Lan thu $try/$MAX_TRIES ==="
    bash scripts/stop_all.sh >/dev/null 2>&1
    ros2 launch tb4_voicenav/launch/voice_demo.launch.py rviz:=false "$@" > "$LOG" 2>&1 &
    LPID=$!
    ok=0
    waited=0
    while [ "$waited" -lt "$ODOM_WAIT_S" ]; do
        sleep 10
        waited=$((waited + 10))
        if ! kill -0 "$LPID" 2>/dev/null; then
            break
        fi
        active=$(grep -c "Managed nodes are active" "$LOG")
        if [ "$active" -ge 2 ] && timeout 8 ros2 topic echo /odom --once >/dev/null 2>&1; then
            ok=1
            break
        fi
        echo "cho... ${waited}s (Nav2 active: ${active}/2)"
    done
    if [ "$ok" -eq 1 ]; then
        echo "SAN SANG: odom co, Nav2 active. Log: $LOG. Ctrl+C de dung."
        wait "$LPID"
        exit 0
    fi
    echo "Chua san sang, dung va thu lai"
    kill -INT "$LPID" 2>/dev/null
    sleep 3
    bash scripts/stop_all.sh >/dev/null 2>&1
done
echo "THAT BAI sau $MAX_TRIES lan. Xem $LOG"
exit 1
