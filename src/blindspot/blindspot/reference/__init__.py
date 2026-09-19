"""Reference controllers - EXAMPLES, not the product.

The library is the guard. These are the implementations the findings were
measured with, kept runnable so the numbers in the README can be reproduced
and so there is something concrete to read. Most people already have a
controller and should keep it; the guard is what plugs in.

Each takes the truncation tolerance `rel_tau` as its own parameter. That is
deliberate: tau belongs to the controller, never to the guard. Sharing one
constant between them was a real bug.
"""

from blindspot.study.ibvs_core import run_ibvs                      # noqa: F401
from blindspot.study.partitioned import run_partitioned             # noqa: F401
from blindspot.study.truncated import run_truncated, pinv_truncated  # noqa: F401
from blindspot.study.switched import run_switched                   # noqa: F401

__all__ = ["run_ibvs", "run_partitioned", "run_truncated", "run_switched",
           "pinv_truncated"]
