# Geometric Tracking Controller

A high-performance geometric tracking controller for quadrotors, designed for precise trajectory following with optimal motor control and frame handling.

## Author and Affiliation

**Developer:**
- Simone D'Angelo - PRISMA Lab, University of Naples Federico II

**Institutional Affiliation:**
- PRISMA Lab (Projects of Robotics for Industry and Service Manipulation)
- Department of Electrical Engineering and Information Technology (DIETI)
- University of Naples Federico II, Italy

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Overview

This controller implements a geometric tracking approach for quadrotor control, featuring:

- **FRD Frame Operation**: Works in Forward-Right-Down coordinate system
- **Direct Motor Control**: Bypasses intermediate control layers for optimal performance  
- **Modular Architecture**: Clean separation between frame conversion and control logic
- **Real-time Performance**: Optimized for 100Hz control loop

## Architecture

### System Integration

```
Gazebo (FLU) → Frame Converter → Controller (FRD) → Motor Commands
```

The controller receives pre-processed FRD odometry data from the frame_converter package, eliminating the need for internal coordinate transformations and improving performance.

### Key Features

- **Lee Geometric Controller**: Mathematically rigorous trajectory tracking
- **Motor Allocation Matrix**: Optimized force/torque to motor speed mapping
- **Adaptive Safety Systems**: Velocity limiting, rate limiting, and NaN/Inf protection
- **Auto-hover Mode**: Automatic position holding at takeoff location
- **Interactive Command Interface**: Real-time trajectory commanding via console input

## Configuration

### Motor Configuration (X-Frame)

Current configuration supports standard X-frame quadrotors:

- **Motor 0** (Front-Right): CCW rotation, angle: 0.615 rad
- **Motor 1** (Back-Left): CCW rotation, angle: 3.975 rad  
- **Motor 2** (Front-Left): CW rotation, angle: 5.668 rad
- **Motor 3** (Back-Right): CW rotation, angle: 2.308 rad

### Key Parameters

```yaml
# Control gains
kp: [7.0, 7.0, 6.0]           # Position gains
kd: [3.0, 3.0, 3.0]           # Velocity gains  
attitude_gain: [0.50, 0.50, 0.15]     # Attitude gains
angular_rate_gain: [0.2, 0.2, 0.18]   # Rate gains

# Vehicle parameters
mass: 0.8704                   # kg
inertia: [0.008951, 0.003102, 0.01161]  # kg·m²
max_motor_velocity: 1885.0     # rad/s

# Motor configuration
rotor_angles: [0.615, 3.975, 5.668, 2.308]      # rad
arm_length: [0.1814, 0.1815, 0.1814, 0.1815]    # m  
motor_rotation_direction: [1.0, 1.0, -1.0, -1.0] # CCW/CW
```

## Usage

### Launch Controller

```bash
# Standard launch with frame converter
ros2 launch geometric_tracking_controller controller.launch.py

# Direct launch (requires pre-converted FRD data)
ros2 run geometric_tracking_controller geometric_tracking_controller
```

### Command Interface

The controller provides an interactive command interface:

```
Insert new coordinates x (front), y (right), z (downward), yaw (clockwise)
> 1.0 0.5 -2.0 0.785  # Move to position and takeoff
```

### Topics

- **Input**: `/quadrotor/odometry_frd` (nav_msgs/Odometry)
- **Output**: `/quadrotor/gazebo/command/motor_speed` (actuator_msgs/Actuators)

## Safety Features

- **Velocity Clamping**: Motor speeds limited to SDF maximum (1885 rad/s)
- **Rate Limiting**: Smooth motor speed transitions (50 rad/s per cycle)
- **Low-pass Filtering**: Alpha=0.8 filter for motor command smoothing  
- **NaN/Inf Protection**: Automatic detection and recovery from invalid control signals
- **Gradual Arming**: Smooth motor spin-up and shutdown sequences

## Integration

Designed to work seamlessly with:

- **frame_converter**: Provides clean FRD odometry data
- **Gazebo Garden**: Direct motor velocity control via MulticopterMotorModel plugin
- **Standard ROS2**: Uses standard message types and conventions

The modular design ensures optimal performance by separating coordinate transformations from the high-frequency control loop.

Dove:
- `k_f`: motor_force_k (1.52117969e-5)
- `k_m`: motor_moment_k (0.016) 
- `l_i`: arm_length[i] * sin/cos(rotor_angles[i])
- Le velocità finali sono ottenute come `ω_i = √(ω_i²)`

## Configurazione

I parametri sono definiti in `config/controller_params.yaml`:

```yaml
geometric_tracking_controller:
  ros__parameters:
    namespace: "quadrotor"  # Deve corrispondere al robotNamespace in SDF
    mass: 0.8704           # Da fpv_drone.sdf
    kp: [5.5, 5.5, 6.0]   # Guadagni posizione [North, East, Down]
    kd: [3.0, 3.0, 4.0]   # Guadagni velocità [North, East, Down]
    motor_num: 4
    # ... altri parametri
```

## Utilizzo

### 1. Avvio simulazione
```bash
ros2 launch quadrotor_gz spawn_quad.launch.py
```

### 2. Avvio controller
```bash
ros2 launch geometric_tracking_controller controller.launch.py
```

### 3. Verifica connessione
```bash
ros2 topic list | grep -E "(odometry|imu|motor_speed)"
ros2 topic echo /model/quadrotor/odometry
```

## Conversione ENU ↔ NED

Il controller implementa automaticamente la trasformazione:

```cpp
// Posizione: [North, East, Down] = [North_ENU, East_ENU, -Up_ENU]
pos_ned.x = pos_enu.y;  // North = North_ENU
pos_ned.y = pos_enu.x;  // East = East_ENU  
pos_ned.z = -pos_enu.z; // Down = -Up_ENU

// Stessa trasformazione per velocità e velocità angolari
```

## Build

```bash
cd sitl_ws
colcon build --packages-select geometric_tracking_controller
source install/setup.bash
```

## Integrazione

Il controller è progettato per funzionare con:
- Modello `fpv_drone` in `quadrotor_description`
- Bridge `ros_gz_bridge` in `quadrotor_gz`
- Mondo `fpv_world.sdf` per gare FPV

Assicurarsi che il `robotNamespace` nel file SDF corrisponda al parametro `namespace` del controller (default: "quadrotor").

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
