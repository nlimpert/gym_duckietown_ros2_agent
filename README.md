# gym_duckietown_ros2_agent

[![ROS Eloquent Elusor][ros-badge-image]][ros-badge-url]
[![Ubuntu 18.04][ubuntu-badge-image]][ubuntu-badge-url]

## Overview
ROS2 implementation of the official [gym-duckietown-ros-agent](https://github.com/duckietown/gym-duckietown-ros-agent.git).

The gym-duckietown-ros2-agent package has been tested under ROS2 Eloquent and Ubuntu 18.04.

## Prerequisites
If you haven't installed ROS2 already, please follow [this tutorial](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) to set up your system. ROS2 is required to use this package.

Besides a working ROS2 installation, the duckietown-gym-daffy package is required. You can install it as described [here](https://github.com/duckietown/gym-duckietown/tree/daffy#installation) or via pip.
```bash
pip3 install duckietown-gym-daffy
```

(optional) To fully leverage the simulation ensure to have the `ros-eloquent-image-transport` and `ros-eloquent-image-transport-plugins` packages installed:
```bash
sudo apt install ros-eloquent-image-transport ros-eloquent-image-transport-plugins
```

## Installing
In order to build this package you have to clone this repository and the duckietown_msgs repository in your workspace.
```bash
cd <ros2-workspace>/src
git clone https://github.com/nlimpert/gym_duckietown_ros2_agent
git clone https://github.com/nlimpert/duckietown_msgs
```

After that's done, go to your workspace folder and build the packages.
```bash
cd <ros2-workspace>
colcon build --symlink-install
```

## Usage
After building the packages and sourcing your workspaces `setup.bash` you can start the agent as follows.
```bash
ros2 run gym_duckietown_ros2_agent rosagent
```
You are now able to receive the camera image via the `/None/corrected_image/compressed` topic. By sending velocity commands for the left and right wheel through the `/None/wheels_driver_node/wheels_cmd` topic, you are able to make the duckiebot move.

To test out you can view the camera image as follows.
```bash
ros2 run rqt_image_view rqt_image_view /None/corrected_image/compressed
```

Using a graphical tool called `rqt_publisher` you can send some velocity commands. To start this tool execute the following command.
```bash
ros2 run rqt_publisher rqt_publisher
```

[ros-badge-image]: https://img.shields.io/badge/ROS2-Eloquent-blue.svg
[ros-badge-url]: https://index.ros.org/doc/ros2/Installation/Eloquent/
[ubuntu-badge-image]: https://img.shields.io/badge/Ubuntu-18.04-orange.svg
[ubuntu-badge-url]: https://www.ubuntu.com/
