## Description
This package synchronizes the velocities of a twist
(represented as a geometry_msgs/TwistStamped message),
such that each component of a Cartesian error (compensated
by the twist's velocities) simultaneously reaches zero.

## Usage
1. Launch the component:
```roslaunch mcr_twist_synchronizer twist_synchronizer.launch```
2. Subscribe to the result of the component:
```rostopic echo /twist_synchronizer/synchronized_twist```
3. Publish the (component-wise) pose difference (example):
```rostopic pub /twist_synchronizer/pose_error mcr_manipulation_msgs/ComponentWiseCartesianDifference '{header: {frame_id: "/arm_link_5"}, linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}'```
4. Publish a twist velocity (example):
```rostopic pub /twist_synchronizer/twist geometry_msgs/TwistStamped '{header: {frame_id: "/arm_link_5"}, twist: {linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}}'```
5. Start the component:
```rostopic pub /twist_synchronizer/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /twist_synchronizer/event_in std_msgs/String  'e_stop'```