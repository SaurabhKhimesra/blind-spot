#!/bin/bash
# The folding-part act: two UR5e cells, one with the guard, one without.
#     ~/blind-spot/run_fold_demo.sh                 # watch it (rqt HUD: view_hud.sh)
#     RECORD=1 RATE=150 ~/blind-spot/run_fold_demo.sh   # record frames for the clip
#
# RATE is physics steps per wall second (500 = real time). Everything runs on
# sim time, so a lower RATE only stretches wall-clock time; it gives the 1080p
# film cameras room to render every frame.
set -e
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH

WS="$HOME/blind-spot/ros2_ws"
OUT="${OUT:-$HOME/blindspot_clip/run_$(date +%H%M%S)}"
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$HOME/blind-spot:$PYTHONPATH"
export ROS_DOMAIN_ID=7

cleanup() {
    # Each child runs in its own process group (setsid), so the whole tree is
    # stopped by group id. A background job in a non-interactive script
    # IGNORES SIGINT, which is why the first version left bridges running.
    for pg in $FOLD $REC $LAUNCH; do kill -TERM -- -$pg 2>/dev/null || true; done
    sleep 5
    for pg in $FOLD $REC $LAUNCH; do kill -KILL -- -$pg 2>/dev/null || true; done
    pkill -f "[p]anel_fold.sdf" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "output: $OUT"
setsid ros2 launch blindspot_arm fold.launch.py gui:=${GUI:-false} rate:=${RATE:-500} \
    > "$OUT/launch.log" 2>&1 &
LAUNCH=$!

echo "waiting for controllers..."
for i in $(seq 1 120); do
    if ros2 control list_controllers -c /left/controller_manager 2>/dev/null | grep -q "arm_controller.*active" \
    && ros2 control list_controllers -c /right/controller_manager 2>/dev/null | grep -q "arm_controller.*active"; then
        echo "controllers active"
        break
    fi
    sleep 2
done

if [ "${RECORD:-0}" = "1" ]; then
    setsid ros2 run blindspot_arm record --ros-args -p out:="$OUT/frames" > "$OUT/record.log" 2>&1 &
    REC=$!
    sleep 3
fi

# In the background and waited on: bash defers a trap until a FOREGROUND
# command returns, so a foreground node that never ends made TERM a no-op.
setsid ros2 run blindspot_arm fold --ros-args -p log:="$OUT/log.csv" \
    -p t_fold:=${T_FOLD:-20.0} -p t_end:=${T_END:-23.5} > "$OUT/fold.log" 2>&1 &
FOLD=$!
wait $FOLD
echo "act complete: $OUT"
