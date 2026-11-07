#!/bin/bash
set -uo pipefail

SPEECH_CORES="0-3"
NAV_CORES="4-7"
GAZEBO_CORES="8-11"
WAIT_TIMEOUT_S=120

wait_for_pids() {
    local pattern="$1"
    local waited=0
    local pids=""
    while [ "$waited" -lt "$WAIT_TIMEOUT_S" ]; do
        pids=$(pgrep -f "$pattern" | tr '\n' ' ')
        if [ -n "$pids" ]; then
            echo "$pids"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo ""
    return 1
}

pin_group() {
    local label="$1"
    local pattern="$2"
    local cores="$3"
    echo "=== Cho tien trinh ${label} (toi da ${WAIT_TIMEOUT_S}s) ==="
    local pids
    pids=$(wait_for_pids "$pattern")
    if [ -z "$pids" ]; then
        echo "LOI: khong thay ${label} sau ${WAIT_TIMEOUT_S}s"
        return 1
    fi
    echo "${label} PID: $pids"
    echo "=== Ghim ${label} vao loi ${cores} (moi luong, -a) ==="
    for pid in $pids; do
        taskset -a -cp "$cores" "$pid"
    done
}

pin_group "speech_node/voice_nav_node" "tb4_voicenav/(speech_node|voice_nav_node)" "$SPEECH_CORES"
pin_group "nav2" "nav2_bringup|bt_navigator|controller_server|planner_server|amcl" "$NAV_CORES"
pin_group "ign gazebo" "ign gazebo" "$GAZEBO_CORES"

echo "=== Hoan tat ghim CPU ==="
