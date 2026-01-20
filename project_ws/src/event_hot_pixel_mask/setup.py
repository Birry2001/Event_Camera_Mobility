from setuptools import setup

package_name = "event_hot_pixel_mask"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hot_pixel_mask.launch.py"]),
        ("share/" + package_name + "/config", ["config/hot_pixel_mask.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="Filter hot pixels in event streams by learning high-rate pixels.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hot_pixel_mask_node = event_hot_pixel_mask.hot_pixel_mask_node:main",
        ],
    },
)
