"""The checks that lock the study's results down.

    regression   every result that took work to establish (ros2 run blindspot regression)
    fd_check     every derivative and sign, by finite difference

Both run under `colcon test` through test/test_checks.py. The guard's own API
and the folding-part act are checked in blindspot_cpp, against the library the
nodes link:

    ros2 run blindspot_cpp api_checks
    ros2 run blindspot_cpp arm_checks
"""
