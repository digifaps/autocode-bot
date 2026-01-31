from setuptools import find_packages, setup

package_name = "motor_driver"

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
    description="Motor driver node for differential drive control using RioRand controllers",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver_node = motor_driver.motor_driver_node:main",
        ],
    },
)
