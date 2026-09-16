#!/bin/bash
# Launches the two-cell Gazebo demo. Run from anywhere:
#     ~/blind-spot/run_arm_demo.sh
#
# Does all the sourcing itself, and deliberately clears LD_LIBRARY_PATH:
# a shell that has sourced the RoboStack conda env will otherwise feed conda
# libraries to system binaries and fail in confusing ways.
set -e
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH

WS="$HOME/blind-spot/ros2_ws"
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$HOME/blind-spot:$PYTHONPATH"
export ROS_DOMAIN_ID=7

echo "starting Gazebo + two UR5e cells..."
ros2 launch blindspot_arm duel.launch.py gui:=${GUI:-true} > /tmp/blindspot_duel.log 2>&1 &
LAUNCH=$!
trap 'kill $LAUNCH 2>/dev/null; pkill -f blindspot_arm 2>/dev/null; exit 0' INT TERM

echo "waiting for controllers (about 40 s)..."
for i in $(seq 1 60); do
    if ros2 control list_controllers -c /left/controller_manager 2>/dev/null | grep -q active \
    && ros2 control list_controllers -c /right/controller_manager 2>/dev/null | grep -q active; then
        echo "controllers active"
        break
    fi
    sleep 2
done

echo "starting the servo + guard loop; HUD on /duel/hud"
echo "view it with:  ros2 run rqt_image_view rqt_image_view /duel/hud"
ros2 run blindspot_arm duel
