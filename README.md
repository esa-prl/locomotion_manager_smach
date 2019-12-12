# locomotion_manager

ROS 1 package that uses the executive SMACH library to create and manage a state machine that handles locomotion mode switching.

For interoperability with ROS 2, rebuild a bridge_ws that contains the appropriate bridge source code after sourcing both the catkin workspace containing this package and the colcon workspace that will be using its functionality. In the ROS 2 message package you will also have to add a `srv_mapping.yaml` file, which should contain the following:

```
-
        ros1_package_name: 'locomotion_manager'
        ros2_package_name: 'locomotion_msgs'
```

Extend this at will as you add more bridged custom service calls or message types.
