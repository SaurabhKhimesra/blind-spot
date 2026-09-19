from setuptools import find_packages, setup

PKG = "blindspot"
SCRIPTS = ["regression", "fd_check", "api_checks", "compare", "tau_sweep",
           "noise_study", "fig_retreat", "fig_failure_modes", "quickstart"]

setup(
    name=PKG,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Saurabh Khimesra",
    maintainer_email="learningkhimesra@gmail.com",
    description="Feature guard for partitioned IBVS, and the study behind it.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": (
        ["calibrate = blindspot.calibrate:main"]
        + ["%s = blindspot.cli:%s" % (s, s) for s in SCRIPTS])},
)
