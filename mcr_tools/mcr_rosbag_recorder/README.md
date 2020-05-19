mcr_rosbag_recorder
===================

This package contains a ROS node which starts and stops a `rosbag record` process based on events published on a topic. This allows you to start/stop recording data from another node or script, which is useful for collecting datasets where you want to trigger the recording based on some event produced by the robot.


## Usage
1. Set up a `yaml` file with a list of the topics you want to record. See [here](ros/config/topics.yaml) for an example.
2. Create a launch file similar to [this](ros/launch/rosbag_recorder.launch) with the appropriate parameters and arguments.
3. Launch the node; for example `roslaunch mcr_rosbag_recorder rosbag_recorder.launch`.
4. When you want to start recording, publish a `std_msgs/String` message with the data `e_start` on the topic `/mcr_tools/rosbag_recorder/event_in`; for example `rostopic pub -1 /mcr_tools/rosbag_recorder/event_in std_msgs/String e_start`
5. Listen to the topic `/mcr_tools/rosbag_recorder/event_out` for a `std_msgs/String` message. If you receive `e_started`, this means the recording started successfully, and if you receive `e_failed`, it means the recording did not start.

## Notes
* multiple instances can be started, but make sure to change the topic remappings and prefix in the launch file as required
* the format for the file name is as follows: `prefix_YYYY_MM_DD_HH-mm.bag`; therefore if you attempt to start two recordings in the same minute with the same prefix, it will fail
* if you want to record a large number of topics, there might be a delay between when the `e_start` event is received and when the recording actually starts; change the `~timeout` parameter as required
* additional `rosbag record` [arguments](https://wiki.ros.org/rosbag/Commandline#record) can be specified as a string with the parameter `~rosbag_arguments` in the launch file
