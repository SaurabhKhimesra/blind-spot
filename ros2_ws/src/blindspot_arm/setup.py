import os
from glob import glob
from setuptools import setup

pkg = "blindspot_arm"
setup(
    name=pkg, version="0.1.0", packages=[pkg],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + pkg]),
        ("share/" + pkg, ["package.xml"]),
        (os.path.join("share", pkg, "urdf"), glob("urdf/*")),
        (os.path.join("share", pkg, "config"), glob("config/*")),
        (os.path.join("share", pkg, "launch"), glob("launch/*")),
        (os.path.join("share", pkg, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Saurabh", maintainer_email="learningkhimesra@gmail.com",
    description="UR5e eye-in-hand servoing under the blindspot guard.",
    license="MIT",
    entry_points={"console_scripts": ["wave = blindspot_arm.wave:main",
            "duel = blindspot_arm.duel_node:main"]},
)
