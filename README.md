# tl_vision2

A ROS2 package to integrate the Vilota vision modules onto PX4

***NOTE:*** Development was carried out on Ubuntu 20.04 Focal and ROS2 Foxy

1. Clone the repo using the following command

```bash
git clone --recurse-submodules https://github.com/navinkaviyarasu/tl_vision2.git
```

2. Build the workspace
- Before building the workspace, install the respective vrpn-mocap package
```bash
sudo apt install ros-<ros-distro>-vrpn-mocap

#Example
sudo apt install ros-humble-vrpn-mocap
```

```bash
cd tl_vision2
colcon build --symlink-install
```

3. Run the vio_bridge node 
```bash
ros2 run vision vio_bridge_px4
```

Dependencies:
- setuptools 59.6.0
- wheel 0.37.1
- pycapnp 1.3.0
- vrpn-mocap

