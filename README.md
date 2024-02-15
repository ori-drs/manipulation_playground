# manipulation_playground

![Devices Currently Implemented](/manipulation_playground_articulated_devices/resources/devices_0.png)

![Devices Currently Implemented](/manipulation_playground_articulated_devices/resources/devices_1.png)

## Introduction

A set of articulated devices implemented as 'robots' in simulation. Static and actuated models for visualization and simulation in RViz and Gazebo.

The devices have the same affordances as their physical counterparts, and can also simulate reactive force/torque in response to being interacted with (for example, the needle_valve exerts more reactive torque as it is twisted further).

The switches and buttons can also be switched on/off much like their physical counterparts; this is simulated using simple Python scripts that load when the devices are spawned.

Currently implemented (left to right, top to bottom):
- VDL_6_20_110_ball_valve
- DN40_globe_valve
- PN16_gate_valve
- needle_valve
- HNF361_safety_switch
- switch
- valve
- button

## How to Use

To spawn a device in Gazebo simulation, simply include the corresponding `..._spawn.launch` file in your ROS launch file, with the device ID and pose as input arguments. The dynamic properties of the device can be set to 'none' (i.e. disabled), 'normal' or 'stiff'. For devices that permit it, continuous rotation can be enabled or disabled (if disabled, the joint is revolute instead).

An example for spawning the needle_valve is provided below:
```xml
<include file="$(find manipulation_playground_articulated_devices)/devices/needle_valve/launch/needle_valve_spawn.launch" >
  <arg name="needle_valve_ID" value="0"/>
  <arg name="x" value="-1.5"/>
  <arg name="y" value="0.2"/>
  <arg name="z" value="0.85"/>
  <arg name="Y" value="-1.5708"/>
  <arg name="P" value="0.0"/>
  <arg name="R" value="1.5708"/>
  <arg name="dynamics" value="normal"/>
  <arg name="continuous" value="true"/>
</include>
```
Multiple devices can be spawned in the same simulation environment; just make sure that duplicate devices of the same type have different IDs.

Note that the dynamic properties of each device have not been tuned to match those of the real device, but rather give a useful representation of reactive torque in simulation. Feel free to tune the PID gains of the controllers as needed for realistic simulation.

Note also that when the dynamic properties are enabled, the devices essentially act as springs. To simulate interactions such as tightening a valve, it is recommended to turn on the device controllers only when the robot is grasping the device and to turn off the controllers otherwise; this way, the device will not snap back to its initial rotation when released.