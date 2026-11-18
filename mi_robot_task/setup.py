from setuptools import setup

package_name = 'mi_robot_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moises Gonzalez',
    maintainer_email='moises@udg.mx',
    description='Cinematica inversa analitica y FSM para manipulador de 4 GDL',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_fsm_node = mi_robot_task.task_fsm_node:main',
        ],
    },
)
