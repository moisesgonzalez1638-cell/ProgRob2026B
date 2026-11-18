from setuptools import setup
import os
from glob import glob

package_name = 'mi_robot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moises Gonzalez',
    maintainer_email='moises@udg.mx',
    description='Percepcion clasica e IA con TF2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = mi_robot_vision.vision_node:main',
        ],
    },
)
