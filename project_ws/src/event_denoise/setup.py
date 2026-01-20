from setuptools import setup

package_name = "event_denoise"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/denoise.launch.py"]),
        ("share/" + package_name + "/config", ["config/denoise.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nochi",
    maintainer_email="nochi@todo.todo",
    description="Denoise event images (count/time/mask).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_denoise_node = event_denoise.denoise_node:main",
        ],
    },
)
