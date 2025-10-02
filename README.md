# tl_vision2

A ROS2 package to integrate the Vilota vision modules onto PX4

***NOTE:*** Development was carried out on Ubuntu 22.04 Focal and ROS2 Humble

1. Clone the repo using the following command

- To clone the main development branch, use

```bash
git clone --recurse-submodules https://github.com/navinkaviyarasu/tl_vision2.git
```
- To clone the installation branch, use
```bash
git clone --branch <tag_name> --single-branch https://github.com/navinkaviyarasu/tl_vision2.git
cd tl_vision2
git submodule update --init --recursive
```
<tag_name> should be based on the PX4 version

2. Build the workspace
- Install the package dependencies
```bash
cd tl_vision2
rosdep install --from-paths src --ignore-src -y
```
- Build the package
```bash
colcon build --symlink-install
```

3. Run the vio_bridge node 
```bash
ros2 run vision vio_bridge_px4
```

To update the submodules according to the latest updates or if you face any issues with the submodules, run the following command
```bash
cd tl_vision2
git submodule update --init --recursive
```

Dependencies:
- setuptools 59.6.0
- wheel 0.37.1
- pycapnp 1.3.0
- vrpn-mocap

