from setuptools import find_packages, setup
from glob import glob
import os

package_name = "narval_description"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
        (os.path.join('share', package_name, 'xacro'), glob(os.path.join('xacro', '*.xacro*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
        (os.path.join('share', package_name, 'nodes'), glob(os.path.join('nodes', '*node.py*')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Piotr Flak",
    maintainer_email="piotrflak@student.agh.edu.pl",
    description="Narval description files",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "narval_state_publisher = nodes.narval_state_publisher_node:main"
        ],
    },
)
