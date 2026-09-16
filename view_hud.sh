#!/bin/bash
# Opens the side-by-side HUD window. Run in a second terminal.
unset LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
source /opt/ros/lyrical/setup.bash
source "$HOME/blind-spot/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=7
exec ros2 run rqt_image_view rqt_image_view /duel/hud
