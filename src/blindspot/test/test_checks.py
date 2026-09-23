"""colcon test entry point: both Python check suites must pass in full.

The API and arm-act suites run as blindspot_cpp tests.
"""

import re
import subprocess
import sys

import pytest

SUITES = [
    ("blindspot.checks.regression", r"(\d+)/(\d+) passed"),
    ("blindspot.checks.fd_check", r"(\d+)/(\d+) finite-difference checks passed"),
]


@pytest.mark.parametrize("module,pattern", SUITES)
def test_suite_passes_in_full(module, pattern):
    run = subprocess.run([sys.executable, "-m", module],
                         capture_output=True, text=True, timeout=1800)
    found = re.findall(pattern, run.stdout)
    assert found, "no summary line from %s:\n%s" % (module, run.stdout[-2000:])
    passed, total = map(int, found[-1])
    failures = [l for l in run.stdout.splitlines() if "FAIL" in l]
    assert run.returncode == 0 and passed == total, "\n".join(failures)
