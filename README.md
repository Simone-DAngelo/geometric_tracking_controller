# lee_controller

## Requirements
- **PX4 Autopilot**: Ensure PX4 for SITL is installed and configured.
- **Gazebo Classic**: Installed and properly configured.
- **Micro XRCE-DDS Agent**: Installed to enable DDS communication.
- **ROS2 Environment**: Installed and workspace properly configured.

## Additional Help and Troubleshooting
For further assistance or if you need help, refer to the [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html).

With dockerfiles: https://github.com/Prisma-Drone-Team/sitl_utils

## 1. Starting the PX4 SITL Simulation in Gazebo Classic
1. Navigate to the PX4 SITL directory:
    ```
    cd /path/to/PX4-Autopilot
    ```
2. Set the topic to stream over ROS2 network modifying the file https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml

3. Start the SITL simulation with the desired model:
    ```
    make px4_sitl gazebo-classic
    ```
    **Important Note for Gazebo Garden Users:**

    If you are using Gazebo Garden instead of Gazebo Classic, be aware that the commands differ. Refer to the user guide for the specific instructions.

## 2. Starting the Micro XRCE-DDS Agent
1. Open a new terminal.
2. Run the following command to start the agent (ensure that any required environment variables are set):
    ```
    MicroXRCEAgent udp4 -p 8888
    ```
3. Verify that the agent is running and listening on the configured port.

## 3. Starting the Developed Node
1. Build the project (if not already compiled):
    ```
    cd /home/dev/ros2_ws
    colcon build --packages-select lee_controller
    ```
2. Source the environment:
    ```
    source install/setup.bash
    ```
3. Run the node:
    ```
    ros2 run lee_controller lee_controller --ros-args --params-file $(path_to_ros2_workspace)/src/lee_controller/conf/iris_param.yaml
    ```
