# Installation
## Requirements
This package is developed for:

    - Ubuntu 20.04
    - ROS 2 foxy

The following installation guide assume that a [ROS 2 workspace](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) is already available.

Also `rosdep2` should be installed:

    sudo apt install python3-rosdep2
### Webots and ROS 2 Interface
Webots and `webots_ros2` are a prerequisite to use the `webots_ros2_robotino3` package. An installation
guide can be found on the [Github repository](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install).

The installation of Webots is straightforward, but if need the installation instructions can be found [here](https://www.cyberbotics.com/doc/guide/installing-webots)

### MPS
The use of the MPS requires several packages. First install [opcua](https://github.com/FreeOpcUa/python-opcua) and
[asyncio](https://github.com/FreeOpcUa/python-opcua):

    sudo apt install python-opcua
    pip install asyncua

Then build the [mps_control](https://git.fh-aachen.de/maskor/mps_control) and [mps_control_actions](https://git.fh-aachen.de/maskor/mps_control_actions)
package, which provides among others Conveyor actions.

The MPS packages are needed to execute the `rcll_world_test.wbt`. If you want the start this world without these packages,
the controller script of the MPS must be set to `void`.



### Apriltags
The `webots_ros2_robotino3` simulation uses `apriltags` which depends on the following packages:
    [AprilTag ROS2 Node](https://github.com/christianrauch/apriltag_ros),
    [apriltag_msgs](https://github.com/christianrauch/apriltag_msgs),
    [apriltag](https://github.com/christianrauch/apriltag).



## Building `webots_ros2_robotino3` package

Now you can build and install the `webots_ros2_robotino3` package.

        cd /path/to/ros2_ws
        git clone https://git.fh-aachen.de/mk9569s/robotino3-webots.git src/
        colcon build

Start the simulation with:

    ros2 launch webots_ros2_robotino3 robot_launch.py

# Navigation

To use navigation start:

    ros2 launch nav2_bringup navigation_launch.py
    ros2 launch slam_toolbox online_async_launch.py