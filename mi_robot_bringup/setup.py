from setuptools import setup
from glob import glob
pkg = 'mi_robot_bringup'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', glob('launch/*.py')),
        ('share/' + pkg + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
)
