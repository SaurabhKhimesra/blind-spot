import os
from glob import glob

from setuptools import setup

PKG = "blindspot_arm"
setup(
    name=PKG,
    version="0.2.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        (os.path.join("share", PKG, "urdf"), glob("urdf/*")),
        (os.path.join("share", PKG, "config"), glob("config/*")),
        (os.path.join("share", PKG, "launch"), glob("launch/*")),
        (os.path.join("share", PKG, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools", "blindspot"],
    zip_safe=True,
    maintainer="Saurabh Khimesra",
    maintainer_email="learningkhimesra@gmail.com",
    description="Two UR5e cells in Gazebo, servoing on a folding part, "
                "with and without the blindspot guard.",
    license="MIT",
    entry_points={"console_scripts": [
        "fold = blindspot_arm.fold_node:main",
        "record = blindspot_arm.record_node:main",
    ]},
)
