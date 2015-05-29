## Description
This package contains components to measure quantitative properties
for different manipulation tasks (e.g. measure the component-wise error
between two poses).


## Usage of 'joint_distance_measurer'
1. Verify that the joint positions to be monitored are available:
```rostopic echo /joint_states```
2. Launch the component (example):
```roslaunch mcr_manipulation_measurers joint_distance_measurer.launch```
**Note**: You will probably need to create your own launch file and configure it according to your needs.
3. Subscribe to the result of the component:
```rostopic echo /joint_distance_measurer/joint_distances```
4. Publish the target joint positions for the required joints (example):
```rostopic pub /joint_distance_measurer/target_joint_values sensor_msgs/JointState "{name: ['arm_1_joint', 'arm_2_joint'], position: [-1.4625053615209804, -1.4834120815495466] }"```
5. Start the component:
```rostopic pub /joint_distance_measurer/event_in std_msgs/String "e_start"```

### To stop the 'joint_distance_measurer':
```rostopic pub /joint_distance_measurer/event_in std_msgs/String "e_stop"```
