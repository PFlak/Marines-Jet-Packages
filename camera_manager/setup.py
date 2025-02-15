from setuptools import find_packages, setup

package_name = 'camera_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominik Wojtasik',
    maintainer_email='dwojtasik@student.agh.edu.pl',
    description='Run all cameras connecting to device',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_manager = camera_manager.camera_manager:main',
        ],
    },
)
