# ros2_control_crospi Package

[**This package is experimental and is under development. It has been tested in simulation (mockup hardware through ros2_control) but not yet on a real robot**]

Generic [Crospi](https://rob.pages.gitlab.kuleuven.be/crospi/) robot driver plugin that interfaces **any robot that has a
[ros2_control](https://control.ros.org) hardware interface** (UR, Franka, KUKA, Kinova, Dynamixel, ...,
`mock_components/GenericSystem`, `topic_based_ros2_control` for Isaac Sim, ...). You don't need to write and
maintain a Crospi driver for every robot brand. Instead, you configure in the setup JSON which robot description
(URDF/xacro with its `<ros2_control>` tag) to use, and the driver loads the hardware plugins declared in it.

## How it works

```mermaid
graph LR
    subgraph "crospi_node"
        CTRL["eTaSL controller"]
    end

    subgraph "ros2_control_crospi plugin (Crospi real-time thread)"
        TB["lock-free triple buffers<br/>(setpoints / feedback)"]
        UPDATE["update():<br/>read() → feedback → setpoints → write()"]
        RM["hardware_interface::ResourceManager<br/>(no controller_manager, no controllers)"]
    end

    subgraph "Robot"
        HW["ros2_control hardware plugin(s)<br/>from the &lt;ros2_control&gt; tag"]
    end

    CTRL <--> TB
    TB <--> UPDATE
    UPDATE --> RM
    RM <--> HW
```

- The robot description is loaded in a `hardware_interface::ResourceManager`, which loads, configures and activates the
  hardware components (ros2_control hardware interface plugins) of its `<ros2_control>` tags.
- There is **no controller_manager and there are no controllers**. Every `periodicity` seconds, Crospi's real-time
  driver thread calls `update()`, which runs one ros2_control cycle directly:
  `read()` of the hardware → feedback written to Crospi's triple buffers → joint velocity setpoints read from Crospi →
  commands set → `write()` of the hardware.
- Crospi's lock-free triple buffers are still used between the eTaSL controller and the driver. ros2_control is only
  used as the hardware abstraction layer, so no mutex-based `realtime_tools` buffers or extra controller layer is
  added to the control path.
- The joint velocity setpoints of Crospi are sent to the **velocity** command interfaces of the joints, or integrated
  into **position** commands (`command_interface`).
- Before the interfaces are claimed, the hardware is asked to switch its control mode
  (`prepare/perform_command_mode_switch`), just as the controller_manager does when a controller is activated
  (e.g. this switches a UR to its velocity mode).
- The plugin is registered both as a **robotdriver** (`simulation:=false`) and as a **robotsimulator**
  (`simulation:=true`), so it can also be used with simulated ros2_control hardware.

## Installation

### Dependencies

The driver needs the ros2_control hardware interface library and xacro. The UR10e quick start below also uses the
mock UR description of `ur_description`:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-ur-description
```

`ros-$ROS_DISTRO-hardware-interface` also provides `mock_components/GenericSystem`, the mock hardware used in the
quick start. Alternatively, `rosdep` installs everything declared in [package.xml](package.xml):

```bash
cd <crospi_ws>
rosdep install --from-paths src --ignore-src -y
```

### Build

Clone this package in the `src` folder of your Crospi workspace and build it:

```bash
cd <crospi_ws>
colcon build --symlink-install --packages-select ros2_control_crospi
source install/setup.bash

# Rebuild your application template package: this scans for new plugins and regenerates the JSON schemas
# of the setup configuration (autocompletion of the parameters below).
colcon build --symlink-install --packages-select <your_crospi_application_template_package>
source install/setup.bash
```

You also need the package that provides the hardware plugin of your robot, e.g. `ur_robot_driver` for UR robots
(`sudo apt install ros-$ROS_DISTRO-ur-robot-driver`). Supported ROS 2 distributions: Jazzy and newer; it has been tested
on Kilted.

## Quick start (UR10e mock hardware, no robot needed)

[config/ur10e_ros2_control_crospi.setup.json](config/ur10e_ros2_control_crospi.setup.json) is a complete Crospi
setup, used for all the examples of this README:

- Its `robotsimulators` entry (`simulation:=true`) uses the UR10e description of `ur_description` with
  `mock_components/GenericSystem`: the velocity commands are integrated by the mock hardware, which also exposes the
  `tcp_fts_sensor` force-torque sensor. The mock hardware starts at the initial joints of
  [config/ur10e_initial_positions.yaml](config/ur10e_initial_positions.yaml).
- Its `robotdrivers` entry (`simulation:=false`) talks to a real UR10e, see
  [Configuring a real robot](#configuring-a-real-robot-ur10e-example).
- Both entries report the joint positions, the joint velocities (`joint_vel`) and the force-torque sensor
  (`wrench_input`) to eTaSL.
- The eTaSL settings and IO handlers are those of the Crospi tutorial (`application_example_tutorials.setup.json`),
  and the robot specification is the UR10e with the cylindrical contact tool of the MuJoCo force-control demos
  (`ur10e_cylinder_tool.urdf`, tcp `tool0`).

```bash
# The plugin alone: loads it through pluginlib (as crospi_node does) and drives all joints with a sinusoidal velocity.
ros2 run ros2_control_crospi test_ros2_control_crospi \
  --config '$[ros2_control_crospi]/config/ur10e_ros2_control_crospi.setup.json' --list robotsimulators --index 0 \
  --duration 4 --amplitude 0.5 --frequency 0.25 --real-time true --csv none

# The full crospi_node with the mock hardware (simulation:=true uses robotsimulators)
ros2 run crospi_core crospi_node --ros-args -p config_file:="\$[ros2_control_crospi]/config/ur10e_ros2_control_crospi.setup.json" -p simulation:=true
```

## Running the Crospi tutorial examples with ros2_control

The first two examples of the [Running examples](https://rob.pages.gitlab.kuleuven.be/crospi/markdown/basic_tutorials/running_examples/)
tutorial can be executed with this driver and the UR10e mock hardware, instead of the `simple_kinematic_simulation`,
using [config/ur10e_ros2_control_crospi.setup.json](config/ur10e_ros2_control_crospi.setup.json). It has the same
eTaSL settings and IO handlers as the setup file of the tutorial (`application_example_tutorials.setup.json`), and
the mock hardware starts at the same initial joints as the tutorial.

Requirements: the [crospi_application_template](https://github.com/Robotics-Research-Group-KUL/crospi_application_template)
and [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) packages in your
workspace (or `sudo apt install ros-$ROS_DISTRO-ur-description`), and the dependencies of [Installation](#installation).
Remember to `source install/setup.bash` in every terminal, and to run the commands from the root of your workspace.

### 1. Execute the launch file with RViz

In a first terminal, launch the robot state publisher and RViz of the tutorial (RViz shows the `/joint_states`
published by the crospi_node, which here come from the ros2_control mock hardware):

```bash
ros2 launch crospi_application_template application_example_tutorials.launch.py
```

### 2. Run the Crospi node

In a second terminal, run the crospi_node with the ros2_control setup file:

```bash
ros2 run crospi_core crospi_node --ros-args -p config_file:="\$[ros2_control_crospi]/config/ur10e_ros2_control_crospi.setup.json" -p simulation:=true
```

The output shows that the hardware `ur` is loaded from the plugin `mock_components/GenericSystem`, configured and
activated, and that the velocity command interfaces of the six joints are claimed. To run the same examples on a
real UR10e, use `simulation:=false` after following [Configuring a real robot](#configuring-a-real-robot-ur10e-example).

### 3. Run the orchestration

#### Example 1

In a third terminal, run the single task that performs a sine wave in the last joint:

```bash
cd src/crospi_application_template/skill_specifications/libraries/skill_lib_example/skill_for_debug_sine_wave_last_joint/
python3 skill_for_debug_sine_wave_last_joint.py
```

The 6th joint moves in a sine wave for 5 s and the script ends with `Runner finished with outcome SUCCEED`.

#### Example 2

In the third terminal, run the finite state machine:

```bash
cd src/crospi_application_template/skill_specifications/libraries/skill_lib_example/skill_example_loop_fsm/
python3 skill_example_loop_fsm.py
```

The robot moves home, down, up and along a spline, and repeats this loop forever (`Ctrl-C` to stop the script).

> **Warning:** 
> (`Ctrl-C`) to stop the robot in the Crospi terminal: the driver then sends a zero velocity command and deactivates and shuts down the
> ros2_control hardware. Only one crospi_node can run at a time.

## Configuration

Add an entry to `robotdrivers` (or `robotsimulators`) in the `robot` field of your setup JSON. The complete reference,
used for autocompletion, is the JSON schema
[json_schemas/ros2_control_crospi.schema.json](json_schemas/ros2_control_crospi.schema.json).

| Parameter | Required | Description |
|-----------|----------|-------------|
| `is-ros2_control_crospi` | yes | `true`: selects this driver. |
| `robot_joints` | yes | Joints controlled by this driver (as in the URDF of the robot specification). |
| `periodicity` | yes | Period [s] of the read/write cycle (e.g. `0.002` for UR e-series, `0.001` for Franka). |
| `robot_description_path` | yes | URDF or `.xacro` file with the `<ros2_control>` tag. Supports `$[package]`, `${ENV}`, `package://` and `file://`. |
| `command_interface` | yes | `velocity` (setpoints sent directly) or `position` (setpoints integrated from the measured initial position). |
| `xacro_arguments` | no | Object with the `name:=value` arguments of xacro, e.g. `{"ur_type": "ur10e", "robot_ip": "192.168.1.102"}`. |
| `hardware_joints` | no | Joint names in the `<ros2_control>` tag if they differ from `robot_joints` (e.g. with a `tf_prefix`). |
| `hardware_components` | no | Hardware components (`<ros2_control name="...">`) to activate. Default: all. |
| `initialization_timeout` | no | Time [s] to wait for valid joint positions after activation. Default: `5.0`. |
| `initialization_settle_time` | no | Minimum time [s] during which the hardware is read after activation before the initial joint positions are taken (some hardware, e.g. UR, reports zeros until its first packets arrive). Default: `0.5`. |
| `ros_node_name` | no | Name of the ROS 2 node of the driver (clock, logging, nodes of the hardware components). Default: `ros2_control_crospi`. Use different names for several drivers. |
| `force_torque_sensor_name` | no | Sensor with `<name>/force.x ... torque.z` state interfaces (e.g. `tcp_fts_sensor` for UR). |
| `pose_sensor_name` | no | Sensor with `<name>/position.x ... orientation.w` state interfaces (e.g. `tcp_pose` for UR). |
| `name_expr_joint_vel` | no | Reports the `velocity` state interfaces of the joints to eTaSL. |
| `name_expr_joint_torque` | no | Reports the `effort` state interfaces of the joints to eTaSL. |
| `name_expr_cartesian_pos` / `name_expr_cartesian_quat` | no | Report the pose sensor (requires `pose_sensor_name`). |
| `name_expr_cartesian_wrench` | no | Reports the force-torque sensor (requires `force_torque_sensor_name`). |

Joint positions are always reported (`position` state interfaces). If a requested interface does not exist in the
hardware, the initialization fails with a message that lists the available interfaces.

### Example: Isaac Sim (`topic_based_ros2_control`)

Use a robot description whose `<ros2_control>` tag uses the
[topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control) plugin (it exchanges the joint
commands and states with Isaac Sim through ROS 2 topics), and add the driver to `robotsimulators`:

```xml
<ros2_control name="isaac" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/isaac_joint_commands</param>
    <param name="joint_states_topic">/isaac_joint_states</param>
  </hardware>
  <joint name="shoulder_pan_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... the other joints ... -->
</ros2_control>
```

## Configuring a real robot (UR10e example)

The setup file of this package, [config/ur10e_ros2_control_crospi.setup.json](config/ur10e_ros2_control_crospi.setup.json),
contains two entries with the same `robot_joints`:

- `robotsimulators` (`simulation:=true`): the UR10e mock hardware (`ur_description/urdf/ur_mocked.urdf.xacro`).
- `robotdrivers` (`simulation:=false`): a real UR10e through the official
  [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), whose hardware plugin
  `ur_robot_driver/URPositionHardwareInterface` is declared in `ur_robot_driver/urdf/ur.urdf.xacro`.

This is the general recipe for any robot: point `robot_description_path` to the robot description of the vendor's
ROS 2 driver (the same URDF/xacro that its launch files give to the controller_manager), pass the arguments of its
launch file as `xacro_arguments`, and choose a `command_interface` that its hardware plugin supports. The steps below
do this for the UR10e. They follow the
[ur_robot_driver documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/), which
describes the robot-side setup in detail.

### 1. Install the hardware plugin

```bash
sudo apt install ros-$ROS_DISTRO-ur-robot-driver   # also installs ur_client_library and ur_description
```

The `robotdrivers` entry references `$[ur_robot_driver]` and `$[ur_client_library]`. If they are not installed
(or the workspace is not sourced), `simulation:=false` fails at initialization with
`could not resolve '$[ur_robot_driver]/urdf/ur.urdf.xacro'`. `simulation:=true` does not need them.

### 2. Prepare the robot and the network

- Connect the PC and the robot controller to the same network, and give the PC a static IP in the robot's subnet
  (e.g. robot `192.168.1.102`, PC `192.168.1.101`).
- On the teach pendant, install the *External Control* URCap (`ros-$ROS_DISTRO-ur-robot-driver` ships it in
  `share/ur_robot_driver/resources/`), set its *Host IP* to the IP of the PC (port `50002`), and create a program
  that contains the *External Control* node.
- Alternatively, set `"headless_mode": "true"` in `xacro_arguments` and put the robot in *Remote Control* mode: the
  hardware plugin then sends the control program itself and no pendant program is needed.
- Optional but recommended: extract the factory calibration of your robot and use it both for ros2_control and in
  the URDF of Crospi's `default_robot_specification`, so that eTaSL uses the exact kinematics of your robot:

  ```bash
  ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.102 \
    target_filename:="${HOME}/ur10e_calibration.yaml"
  ```

  and add `"kinematics_params": "${HOME}/ur10e_calibration.yaml"` to `xacro_arguments`.

### 3. Configure the `robotdrivers` entry

This is the `robotdrivers` entry of
[config/ur10e_ros2_control_crospi.setup.json](config/ur10e_ros2_control_crospi.setup.json). Change at least
`robot_ip`:

```json
"robotdrivers": [
    {
        "is-ros2_control_crospi": true,
        "robot_joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        "periodicity": 0.002,
        "robot_description_path": "$[ur_robot_driver]/urdf/ur.urdf.xacro",
        "xacro_arguments": {
            "name": "ur",
            "ur_type": "ur10e",
            "robot_ip": "192.168.1.102",
            "script_filename": "$[ur_client_library]/resources/external_control.urscript",
            "input_recipe_filename": "$[ur_robot_driver]/resources/rtde_input_recipe.txt",
            "output_recipe_filename": "$[ur_robot_driver]/resources/rtde_output_recipe.txt"
        },
        "command_interface": "velocity",
        "hardware_components": ["ur"],
        "initialization_timeout": 10.0,
        "ros_node_name": "ros2_control_crospi_driver",
        "force_torque_sensor_name": "tcp_fts_sensor",
        "name_expr_joint_vel": "joint_vel",
        "name_expr_cartesian_wrench": "wrench_input"
    }
]
```

- `periodicity: 0.002`: the UR e-series controller communicates at 500 Hz (use `0.008` for CB3 robots). The driver
  thread runs independently of the eTaSL `sample_time` (`0.01`), thanks to Crospi's triple buffers.
- `robot_description_path` and `xacro_arguments`: the description and the arguments that `ur_robot_driver`'s launch
  file (`ur_control.launch.py`) passes to xacro. `script_filename` and the RTDE recipes are required by the hardware
  plugin. Any other argument of `ur.urdf.xacro` can be added (e.g. `headless_mode`, `kinematics_params`,
  `reverse_ip`, `tf_prefix` with `hardware_joints`).
- `command_interface: velocity`: the UR hardware plugin switches the robot to velocity control (speedj) when the
  velocity interfaces are claimed.
- `force_torque_sensor_name` / `name_expr_cartesian_wrench`: the wrist force-torque sensor of the e-series is reported to
  eTaSL as `wrench_input`, the same name that the simulated wrench uses in the MuJoCo force-control demos. It can also
  report the TCP pose computed by the robot with `"pose_sensor_name": "tcp_pose"` and `name_expr_cartesian_pos/quat`.
  The `FT_sensor_frame` of the robot specification is mounted at `tool0`. Before running force control on the real
  robot, check that this matches the frame in which `ur_robot_driver` reports the wrench of `tcp_fts_sensor` (it
  depends on the TCP configured on the teach pendant), together with the weight and center of gravity of the tool
  (see the sim-to-real gap section linked below).

### 4. Run

Start the crospi_node with `simulation:=false`:

```bash
ros2 run crospi_core crospi_node --ros-args -p config_file:="\$[ros2_control_crospi]/config/ur10e_ros2_control_crospi.setup.json" -p simulation:=false
```

The output shows that the hardware `ur` is loaded from `ur_robot_driver/URPositionHardwareInterface`, activated
(connection to the robot at `robot_ip`) and switched to the velocity interfaces, followed by
`Initialized ros2_control robot driver ... initial joints [...]` with the actual joint positions of the robot.

**Then start the program with the *External Control* node on the teach pendant** (press play; not needed in headless
mode). Wait until the crospi_node terminal shows:

```
Robot control program is running and connected to the driver
```

Only then run an orchestration, starting with the safest one, [Example 1](#example-1) (sine wave in the last joint),
before running [Example 2](#example-2) or your own applications. Keep the emergency stop at hand.

### 5. Troubleshooting: the task runs but the robot does not move

The UR hardware plugin only sends the joint commands to the robot while the *External Control* program is playing
and connected to the driver. Otherwise it drops them silently: the eTaSL task executes and finishes normally, but
the robot does not move. If the message `Robot control program is running and connected to the driver` does not
appear:

- Press play on the program with the *External Control* node **after** the crospi_node has printed
  `Initialized ros2_control robot driver`. A program started before the driver cannot connect and stops.
- The *Host IP* of the *External Control* URCap (Installation tab → URCaps → External Control) must be the IP of the
  PC **as seen from the robot**. E.g. for URSim at `192.168.56.102` on a host-only network, it is the address of the PC
  on that network (often `192.168.56.1`, check with `ip addr`), not `127.0.0.1` or the address of another interface.
- The robot must reach the PC on the ports `50001`–`50004` (`reverse_port`, `script_sender_port`, `trajectory_port`,
  `script_command_port`): check the firewall of the PC.
- The robot must be powered on, its brakes released, and not in protective stop. In headless mode
  (`"headless_mode": "true"`), it must be in *Remote Control* mode.

Two other messages of the log are not the cause of this problem, but are worth fixing:

- `The calibration parameters of the connected robot don't match the ones from the given kinematics config file`:
  extract the calibration of the robot and pass it as `kinematics_params` (step 2).
- `Failed to set Thread scheduling : Operation not permitted` (crospi_node) and `Your system/user seems not to be setup
  for FIFO scheduling` (UR client library): the driver thread does not get real-time priority, which causes jitter at
  `periodicity: 0.002`. Allow real-time scheduling for your user (e.g. `sudo groupadd realtime`,
  `sudo usermod -aG realtime $USER`, add `@realtime - rtprio 99` to `/etc/security/limits.conf`, and log in again),
  preferably with a lowlatency
  or PREEMPT_RT kernel, see the
  [ur_client_library real-time setup](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/real_time.html).

> **Note:** the setup file of this package is an example. In your own application, add the `ros2_control_crospi`
> entries to the setup file of your application (e.g. in your `crospi_application_template`), together with the
> robot specification and IO handlers that your tasks need. Before running force-control applications on the real
> robot, read the [sim-to-real gap](https://rob.pages.gitlab.kuleuven.be/crospi/markdown/basic_tutorials/running_examples/#6-sim-to-real-gap)
> section of the Crospi tutorial.

## Things to know

- **Do not run a controller_manager for the same hardware at the same time**: this driver owns the hardware.
  Joint states are published by Crospi's output handlers (e.g. `JointStateOutputHandler`).
- ros2_control **controllers and broadcasters are not executed**. Hardware features that are only reachable through
  controllers or GPIO command interfaces (e.g. UR's `resend_robot_program`, grippers controlled by a controller) need
  to be handled separately.
- The `read()`/`write()` of the hardware run in Crospi's real-time thread. The real-time behavior therefore depends on
  the hardware plugin, in the same way as with the controller_manager.
- Joint limits are not enforced by the driver. Use joint limit constraints in your eTaSL task specification.
- If initialization fails (wrong robot description, unreachable robot, missing interface, ...), the driver prints the
  reason and stops Crospi's control loop at its first update.
- On shutdown, the driver sends a standstill command (zero velocity, or the last position command), switches off the
  command interfaces and deactivates and shuts down the hardware components.

## Documentation

For documentation and tutorials, checkout our [crospi website](https://rob.pages.gitlab.kuleuven.be/crospi/), in
particular the [robot drivers tutorial](https://rob.pages.gitlab.kuleuven.be/crospi/markdown/advanced_tutorials/robot_drivers/).


## License and acknowledgements

Published under the [GNU LESSER GENERAL PUBLIC LICENSE](LICENSE) Version 3, 29 June 2007.


<a href="https://aiprism.eu/"><img src="./Ai-Prism_Logo_Horizontal.png" alt="AI-PRISM Logo" width="150" /></a>
This work was funded by the European Union’s Horizon 2020 research and innovation program 
under the grant agreement No. <a href="https://cordis.europa.eu/project/id/101058589">101058589</a> ( <a href="https://aiprism.eu/">AI-Prism</a>) 

<p/>

<p/>

<p/>

## Authors
<p float="left">
<a href="https://www.kuleuven.be/english/kuleuven/">
    <img src="./logo_kuleuven.png" alt="KU Leuven Logo" width="150"/>
</a>
<a href="https://www.mech.kuleuven.be/en/research/ram">
    <img src="./logo_RAM.png" alt="RAM Logo" width="110" />
</a>
</p>

(c) 2025, KU Leuven, Department of Mechanical Engineering, ROB-Group:

- [Santiago Iregui Rincon](https://www.kuleuven.be/wieiswie/en/person/00125886)
