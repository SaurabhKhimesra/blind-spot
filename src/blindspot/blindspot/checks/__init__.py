"""The checks that lock the results down.

    regression   every result that took work to establish (ros2 run blindspot regression)
    fd_check     every derivative and sign, by finite difference
    api          the packaged API and the ROS bridge, including what they refuse

All three run under `colcon test` through test/test_checks.py.
"""
