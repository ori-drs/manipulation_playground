# manipulation_playground
A set of manipulation models for actuated devices used in the lab. Static and actuated models for visualisation and simulation.

To spawn a model in Gazebo simulation, simply include the corresponding `..._spawn.launch` file in your launch file, with the model ID and pose as input arguments. An example for spawning a button is provided below:
```xml
<include file="$(find manipulation_playground)/devices/button/launch/button_spawn.launch" >
  <arg name="button_ID" value="0"/>
  <arg name="x" value="-1.5"/>
  <arg name="y" value="0.2"/>
  <arg name="z" value="0.85"/>
  <arg name="Y" value="0.0"/>
  <arg name="P" value="1.5707"/>
  <arg name="R" value="0.0"/>
</include>
```