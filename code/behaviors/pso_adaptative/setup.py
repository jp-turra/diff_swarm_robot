from glob import glob
from setuptools import find_packages, setup

package_name = "pso_adaptative"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "functools", "numpy"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="joao.t06@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_protection_layer_us = pso_adaptative.hardware_protection_layer_us:main",
            "pso_adaptative_movement = pso_adaptative.pso_adaptative_movement:main",
        ],
    },
)
