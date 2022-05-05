from glob import glob
from setuptools import setup

package_name = 'webots_ros2_robotino3'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/robot_launch.py']))
data_files.append(('share/' + package_name, ['launch/robot_launch_three.py']))
data_files.append(('share/' + package_name, ['launch/robo_full_launch.py']))

data_files.append(('share/' + package_name + '/controllers/puck_supervisor', [
    'controllers/puck_supervisor/puck_supervisor.py']))

data_files.append(('share/' + package_name + '/controllers/mps_controller', [
    'controllers/mps_controller/mps_controller.py']))

data_files.append(('share/' + package_name + '/protos', [
    'protos/Robotino3.proto'
]))

data_files.append(('share/' + package_name + '/protos', [
    'protos/ConveyorBelt.proto'
]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/ConveyorPlatform.proto'
]))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/robotino3.wbt','worlds/robotino3_backfinger.wbt', 'worlds/rcll_world.wbt','worlds/rcll_world_test.wbt', 'worlds/rcll_world_test_3.wbt'
]))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/tag16_05_00000.png')))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/*.jpg')))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/*.png')))

data_files.append(('share/' + package_name + '/resource', [
    'resource/sf_mps.DeliveryStation.Application.xml'
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/params', [
    'params/webots_params.yaml'
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools','launch'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maximillian.kirsch@alumni.fh-aachen.de',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],    
    description='Robotino3 Webots Driver',
    license='Beer License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotino3_driver = webots_ros2_robotino3.robotino3_driver:main',
            'robotino3_driver_new = webots_ros2_robotino3.robotino3_driver_new:main',
            'robotino3_driver_three = webots_ros2_robotino3.robotino3_driver_three:main',
            'robotino3_driver_four = webots_ros2_robotino3.robotino3_driver_four:main',
            'frame_listener = webots_ros2_robotino3.frame_listener:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
