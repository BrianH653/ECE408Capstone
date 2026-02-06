from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ht301_thermal_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/ht301_thermal_ros']),
    ('share/ht301_thermal_ros', ['package.xml']),
    (os.path.join('share', 'ht301_thermal_ros', 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian_holland',
    maintainer_email='brian_holland@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ht301_thermal_publisher = ht301_thermal_ros.ht301_thermal_publisher:main',
        ],
    },
)
