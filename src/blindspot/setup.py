from setuptools import find_packages, setup

PKG = "blindspot"
SCRIPTS = ["regression", "fd_check", "compare", "tau_sweep",
           "noise_study", "fig_retreat", "fig_failure_modes"]

setup(
    name=PKG,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Saurabh Khimesra",
    maintainer_email="learningkhimesra@gmail.com",
    description=("The IBVS degeneracy study and the checks that lock its "
                 "results. The guard itself is C++, in blindspot_cpp."),
    license="MIT",
    # The checks and the results table are numpy only; only the two figure
    # scripts need matplotlib.
    extras_require={"test": ["pytest"], "figures": ["matplotlib"]},
    entry_points={
        "console_scripts": ["%s = blindspot.cli:%s" % (s, s) for s in SCRIPTS],
    },
)
