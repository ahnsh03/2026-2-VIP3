#!/usr/bin/env python3

from setuptools import find_packages, setup

setup(
    name="vip3_vehicle_state",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
