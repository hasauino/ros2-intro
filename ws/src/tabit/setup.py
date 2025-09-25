import glob
from setuptools import find_packages, setup

package_name = "tabit"
launch_files = glob.glob("launch/*.py")
include_launch_files = glob.glob("launch/include/*")
config_files = glob.glob("configs/*")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", launch_files),
        ("share/" + package_name + "/launch/include", include_launch_files),
        ("share/" + package_name + "/configs", config_files),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
    ],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = tabit.01_talker:main",
            "listener = tabit.01_listener:main",
            "robot_commander = tabit.02_robot_commander:main",
            "navigator = tabit.03_navigator:main",
            "twist_to_twist_stamped = tabit.utils.twist_to_twist_stamped:main",
        ],
    },
)
