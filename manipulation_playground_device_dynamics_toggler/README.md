# manipulation_playground/device_dynamics_toggler

An (optional) package that allows for more realistic simulation of valves when their dynamic properties are enabled. It is used to prevent valve handles from snapping back to their initial joint states when released, instead having them remain at their current positions.

If you want to use this functionality, run the following before manipulating any valves:

`rosrun manipulation_playground_device_dynamics_toggler device_dynamics_toggler.py`

This node provides a ROS service `toggleDeviceDynamics` that can be invoked to set a device's controller command to either its current joint state or its initial joint state. The node can manage multiple devices at once; the device type and ID are arguments in each service call.

To set a device's controller command to its initial joint state (so the device handle will snap back to its initial position), set `enable=true`. Do this just after grasping the valve handle (but before twisting). For example, in command line:

`rosservice call /toggleDeviceDynamics "device_type: 'needle_valve' device_id: '0' enable: true"`

**Note:** The above service must be called even on the first grasp; this tells the device dynamics toggler what the device's initial joint state is.

To set a device's controller command to its current joint state (so the device handle will hold its current position), set `enable=false`. Do this just before releasing the valve handle (but after twisting). For example, in command line:

`rosservice call /toggleDeviceDynamics "device_type: 'needle_valve' device_id: '0' enable: false"`

Rather than using command line, it is recommended to add these ROS service calls into your task planner, just after grasping and just before releasing the valve handle.