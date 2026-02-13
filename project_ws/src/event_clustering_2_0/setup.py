from setuptools import setup

package_name = "event_clustering_2_0"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/clustering_2_0.launch.py"]),
        ("share/" + package_name + "/config", ["config/clustering_2_0.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="Event-based moving object segmentation with dynamic threshold and joint clustering.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_clustering_2_0_node = event_clustering_2_0.clustering_2_0_node:main",
        ],
    },
)
