from setuptools import find_packages, setup

package_name = 'turtlebot3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timotius',
    maintainer_email='timotius@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_turtlebot3 = turtlebot3_control.move_turtlebot3:main',
            'position_reporter = turtlebot3_control.position_reporter_node:main',
        ],
    },
)
