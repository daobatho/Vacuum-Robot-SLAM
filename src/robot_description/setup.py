from setuptools import find_packages, setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package share directory
        ('share/' + package_name, [
            'package.xml',
            'urdf/robot.urdf',
            'launch/default.rviz'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tho',
    maintainer_email='tho@todo.todo',
    description='Robot description package including URDF and RViz config',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
