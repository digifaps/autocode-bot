from setuptools import find_packages, setup

package_name = "lidar_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autocode-bot",
    maintainer_email="todo@todo.com",
    description="Power and scan-speed control for D500 LiDAR (GPIO power enable)",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_control_node = lidar_control.lidar_control_node:main",
        ],
    },
)
