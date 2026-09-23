"""Console entry points: run a study script or a check as `ros2 run blindspot <name>`."""

import runpy


def _run(module):
    def main():
        runpy.run_module(module, run_name="__main__")
    return main


regression = _run("blindspot.checks.regression")
fd_check = _run("blindspot.checks.fd_check")
compare = _run("blindspot.study.compare")
tau_sweep = _run("blindspot.study.tau_sweep")
noise_study = _run("blindspot.study.noise_study")
fig_retreat = _run("blindspot.study.fig_retreat")
fig_failure_modes = _run("blindspot.study.fig_failure_modes")
