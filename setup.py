from glob import glob

from setuptools import find_packages, setup

pkg = "ros2_amr_bench"
setup(
    name=pkg,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{pkg}"]),
        (f"share/{pkg}", ["package.xml"]),
        (f"share/{pkg}/launch", glob("launch/*.py")),
        (f"share/{pkg}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "python-can"],
    zip_safe=True,
    license="MIT",
    entry_points={"console_scripts": [
        f"amr_sim = {pkg}.sim_node:main",
        f"can_bridge = {pkg}.can_bridge_node:main",
        f"waypoint_follower = {pkg}.follower_node:main",
        f"amr_diagnostics = {pkg}.diagnostics_node:main",
    ]},
)
