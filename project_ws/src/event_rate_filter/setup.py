from setuptools import setup

package_name = "event_rate_filter"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rate_filter.launch.py"]),
        ("share/" + package_name + "/config", ["config/rate_filter.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="Filter events by per-pixel rate (EMA) to suppress bright/noisy regions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_rate_filter_node = event_rate_filter.rate_filter_node:main",
        ],
    },
)
