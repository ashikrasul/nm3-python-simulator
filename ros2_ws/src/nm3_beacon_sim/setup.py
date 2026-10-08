from glob import glob

from setuptools import setup

package_name = "nm3_beacon_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/models", glob("models/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Live NM3 beacon localisation (Gazebo sim or real modems) with LS trilateration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pinger_node = nm3_beacon_sim.pinger_node:main",
            "bag_to_csv = nm3_beacon_sim.bag_to_csv:main",
        ],
    },
)
