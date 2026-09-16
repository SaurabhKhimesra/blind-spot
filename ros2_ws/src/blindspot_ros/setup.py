from setuptools import setup

package_name = "blindspot_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py"]),
    ],
    install_requires=["setuptools", "blindspot"],
    zip_safe=True,
    maintainer="Saurabh",
    maintainer_email="learningkhimesra@gmail.com",
    description="Per-step guard for image-based visual servoing.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "guard_node = blindspot_ros.guard_node:main",
            "fake_detections = blindspot_ros.fake_detections:main",
        ],
    },
)
