from setuptools import find_packages, setup
import glob

package_name = 'biped_mujoco_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/mjc_ros2_launch.py']),
        ('share/' + package_name + '/mjcf', ['mjcf/Bipedal_Robot.xml']),
        ('share/' + package_name + '/mjcf', glob.glob('mjcf/*.xml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vanthanhnguyen',
    maintainer_email='thanhnguyenvan0596@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],


entry_points={
        'console_scripts': [
                'mujoco_node = biped_mujoco_ros2.mujoco_node:main',
        ],
},

)
