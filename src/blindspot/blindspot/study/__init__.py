"""The control study: the controllers and the scripts that produced the numbers.

    ibvs_core    SE(3), the pinhole camera, the interaction matrix, classic IBVS,
                 the sigma_6 health metric, the benchmark scenarios
    partitioned  partitioned IBVS (Corke & Hutchinson, IEEE T-RA 2001)
    truncated    the adaptive-rank truncated pseudo-inverse
    switched     the runtime switch, its six candidate rules, and calibration
    predict      forward rollout of the guard signal

    compare, tau_sweep, noise_study, fig_retreat, fig_failure_modes
                 study scripts, each runnable with `ros2 run blindspot <name>`
"""
