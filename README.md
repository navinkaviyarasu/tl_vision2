# tl_vision2

A ROS2 package to integrate the Vilota vision modules onto PX4

***NOTE:*** Development was carried out on Ubuntu 22.04 Focal and ROS2 Humble

1. Clone the repo using the following command

- To clone the development branch(master), use

    ```bash
    git clone --recurse-submodules https://github.com/navinkaviyarasu/tl_vision2.git
    ```
- To clone the installation branch, use
    ```bash
    git clone --branch <tag_name> --single-branch https://github.com/navinkaviyarasu/tl_vision2.git
    cd tl_vision2
    git submodule update --init --recursive
    ```
    NOTE: <tag_name> should be based on the PX4 version

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

3. Running the vio_bridge node 

- To run just the vio_bridge node only

    ```bash
    ros2 run vision vio_bridge_px4
    ```
- To run just the vio_bridge node with namespace

    ```bash
    ros2 run package_name node_executable_name --ros-args -r __ns:=/my_desired_namespace

    ros2 run vision vio_bridge_px4 --ros-args -r __ns:=/my_desired_namespace
    ```
- To run the vio_bridge, Offboard control to fly an autonomous trajectory and RVIZ visualizer

    ```bash
    ros2 launch launcher vision.launch.py namespace:=my_desired_namespace
    ```
- To run Vicon for EKF Fusion and VIO for reference
    ```bash
    ros2 launch launcher vicon.launch.py namespace:=my_desired_namespace mocap_use:=1 vicon_object_name:my_object_name
    ```

    ```bash
    ros2 run vision vio_bridge_px4 --ros-args -r __ns:=/my_desired_namespace -p sensor_use:=2
    ```

4. Updating Submodules

- To update the submodules according to the latest updates or if you face any issues with the submodules, run the following command
    ```bash
    cd tl_vision2
    git submodule update --init --recursive
    ```

Dependencies:
- setuptools 59.6.0
- wheel 0.37.1
- pycapnp 1.3.0
- vrpn-mocap

