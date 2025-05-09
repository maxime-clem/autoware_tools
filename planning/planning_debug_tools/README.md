# Planning Debug Tools

This package contains several planning-related debug tools.

- **Trajectory analyzer**: visualizes the information (speed, curvature, yaw, etc) along the trajectory
- **Closest velocity checker**: prints the velocity information indicated by each modules
- **Perception reproducer**: generates detected objects from rosbag data in planning simulator environment
- **processing time checker**: displays processing_time of modules on the terminal
- **logging level updater**: updates the logging level of the planning modules.

## Trajectory analyzer

The `trajectory_analyzer` visualizes the information (speed, curvature, yaw, etc) along the trajectory. This feature would be helpful for purposes such as "_investigating the reason why the vehicle decelerates here_". This feature employs the OSS [PlotJuggler](https://www.plotjuggler.io/).

![how this works](https://user-images.githubusercontent.com/21360593/179361367-a9fa136c-cd65-4f3c-ad7c-f542346a8d37.mp4)

## Stop reason visualizer

This is to visualize stop factor and reason.
[see the details](./doc-stop-reason-visualizer.md)

### How to use

please launch the analyzer node

```bash
ros2 launch planning_debug_tools trajectory_analyzer.launch.xml
```

and visualize the analyzed data on the plot juggler following below.

#### setup PlotJuggler

For the first time, please add the following code to reactive script and save it as the picture below!
(Looking for the way to automatically load the configuration file...)

You can customize what you plot by editing this code.

![image](./image/lua.png)

in Global code

```lua
behavior_path = '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id/debug_info'
behavior_velocity = '/planning/scenario_planning/lane_driving/behavior_planning/path/debug_info'
motion_avoid = '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory/debug_info'
motion_smoother_latacc = '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_lateral_acc_filtered/debug_info'
motion_smoother = '/planning/scenario_planning/trajectory/debug_info'
```

in function(tracker_time)

```lua
PlotCurvatureOverArclength('k_behavior_path', behavior_path, tracker_time)
PlotCurvatureOverArclength('k_behavior_velocity', behavior_velocity, tracker_time)
PlotCurvatureOverArclength('k_motion_avoid', motion_avoid, tracker_time)
PlotCurvatureOverArclength('k_motion_smoother', motion_smoother, tracker_time)

PlotVelocityOverArclength('v_behavior_path', behavior_path, tracker_time)
PlotVelocityOverArclength('v_behavior_velocity', behavior_velocity, tracker_time)
PlotVelocityOverArclength('v_motion_avoid', motion_avoid, tracker_time)
PlotVelocityOverArclength('v_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotVelocityOverArclength('v_motion_smoother', motion_smoother, tracker_time)

PlotAccelerationOverArclength('a_behavior_path', behavior_path, tracker_time)
PlotAccelerationOverArclength('a_behavior_velocity', behavior_velocity, tracker_time)
PlotAccelerationOverArclength('a_motion_avoid', motion_avoid, tracker_time)
PlotAccelerationOverArclength('a_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotAccelerationOverArclength('a_motion_smoother', motion_smoother, tracker_time)

PlotYawOverArclength('yaw_behavior_path', behavior_path, tracker_time)
PlotYawOverArclength('yaw_behavior_velocity', behavior_velocity, tracker_time)
PlotYawOverArclength('yaw_motion_avoid', motion_avoid, tracker_time)
PlotYawOverArclength('yaw_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotYawOverArclength('yaw_motion_smoother', motion_smoother, tracker_time)

PlotCurrentVelocity('localization_kinematic_state', '/localization/kinematic_state', tracker_time)
```

in Function Library
![image](./image/script.png)

<!-- cspell:ignore Timeseries -->
<!-- Ignore cspell errors caused by external factors -->

```lua

function PlotValue(name, path, timestamp, value)
  new_series = ScatterXY.new(name)
  index = 0
  while(true) do
    series_k = TimeseriesView.find( string.format( "%s/"..value.."[%d]", path, index) )
    series_s = TimeseriesView.find( string.format( "%s/arclength[%d]", path, index) )
    series_size = TimeseriesView.find( string.format( "%s/size", path) )

    if series_k == nil or series_s == nil then break end

    k = series_k:atTime(timestamp)
    s = series_s:atTime(timestamp)
    size = series_size:atTime(timestamp)

    if index >= size then break end

    new_series:push_back(s,k)
    index = index+1
  end
end

function PlotCurvatureOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"curvature")
end

function PlotVelocityOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"velocity")
end

function PlotAccelerationOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"acceleration")
end

function PlotYawOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"yaw")
end

function PlotCurrentVelocity(name, kinematics_name, timestamp)
  new_series = ScatterXY.new(name)
  series_v = TimeseriesView.find( string.format( "%s/twist/twist/linear/x", kinematics_name))
  if series_v == nil then
    print("error")
    return
  end
  v = series_v:atTime(timestamp)
  new_series:push_back(0.0, v)
end
```

Then, run the plot juggler.

### How to customize the plot

Add Path/PathWithLaneIds/Trajectory topics you want to plot in the `trajectory_analyzer.launch.xml`, then the analyzed topics for these messages will be published with `TrajectoryDebugINfo.msg` type. You can then visualize these data by editing the reactive script on the PlotJuggler.

### Requirements

The version of the plotJuggler must be > `3.5.0`

## Closest velocity checker

This node prints the velocity information indicated by planning/control modules on a terminal. For trajectories calculated by planning modules, the target velocity on the trajectory point which is closest to the ego vehicle is printed. For control commands calculated by control modules, the target velocity and acceleration is directly printed. This feature would be helpful for purposes such as "_investigating the reason why the vehicle does not move_".

You can launch by

```bash
ros2 run planning_debug_tools closest_velocity_checker.py
```

![closest-velocity-checker](image/closest-velocity-checker.png)

## Trajectory visualizer

The old version of the trajectory analyzer. It is written in Python and more flexible, but very slow.

## For other use case (experimental)

To see behavior velocity planner's internal plath with lane id
add below example value to behavior velocity analyzer and set `is_publish_debug_path: true`

```lua
crosswalk ='/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path_with_lane_id/crosswalk/debug_info'
intersection ='/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path_with_lane_id/intersection/debug_info'
traffic_light ='/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path_with_lane_id/traffic_light/debug_info'
merge_from_private ='/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path_with_lane_id/merge_from_private/debug_info'
occlusion_spot ='/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/path_with_lane_id/occlusion_spot/debug_info'
```

```lua
PlotVelocityOverArclength('v_crosswalk', crosswalk, tracker_time)
PlotVelocityOverArclength('v_intersection', intersection, tracker_time)
PlotVelocityOverArclength('v_merge_from_private', merge_from_private, tracker_time)
PlotVelocityOverArclength('v_traffic_light', traffic_light, tracker_time)
PlotVelocityOverArclength('v_occlusion', occlusion_spot, tracker_time)

PlotYawOverArclength('yaw_crosswalk', crosswalk, tracker_time)
PlotYawOverArclength('yaw_intersection', intersection, tracker_time)
PlotYawOverArclength('yaw_merge_from_private', merge_from_private, tracker_time)
PlotYawOverArclength('yaw_traffic_light', traffic_light, tracker_time)
PlotYawOverArclength('yaw_occlusion', occlusion_spot, tracker_time)

PlotCurrentVelocity('localization_kinematic_state', '/localization/kinematic_state', tracker_time)
```

## Perception reproducer

This script can overlay the perception results from the rosbag on the planning simulator synchronized with the simulator's ego pose.

### How it works

Whenever the ego's position changes, a chronological `reproduce_sequence` queue is generated based on its position with a search radius (default to 2 m).
If the queue is empty, the nearest odom message in the rosbag is added to the queue.
When publishing perception messages, the first element in the `reproduce_sequence` is popped and published.

This design results in the following behavior:

- When ego stops, the perception messages are published in chronological order until queue is empty.
- When the ego moves, a perception message close to ego's position is published.

### Available Options

- `-b`, `--bag`: Rosbag file path (required)
- `-d`, `--detected-object`: Publish detected objects
- `-t`, `--tracked-object`: Publish tracked objects
- `-r`, `--search-radius`: Set the search radius in meters (default: 1.5m). If set to 0, always publishes the nearest message
- `-c`, `--reproduce-cool-down`: Set the cool down time in seconds (default: 80.0s)
- `-p`, `--pub-route`: Initialize localization and publish a route based on poses from the rosbag
- `-n`, `--noise`: Apply perception noise to objects when publishing repeated messages (default: True)
- `-f`, `--rosbag-format`: Specify rosbag data format (default: "db3")
- `-v`, `--verbose`: Output debug data

### How to use

First, launch the planning simulator, and put the ego pose.
Then, run the script according to the following command.

By designating a rosbag, perception reproducer can be launched.

```bash
ros2 run planning_debug_tools perception_reproducer.py -b <bag-file>
```

You can designate multiple rosbags in the directory.

```bash
ros2 run planning_debug_tools perception_reproducer.py -b <dir-to-bag-files>
```

Instead of publishing predicted objects, you can publish detected/tracked objects by designating `-d` or `-t`, respectively.

The `--pub-route` option enables automatic route generation based on the rosbag data. When enabled, the script:

1. Extracts the initial and goal poses from the beginning and end of the rosbag file
2. Initializes the localization system with the initial pose
3. Generates and publishes a route to the goal pose

Example usage with route publication:

```bash
ros2 run planning_debug_tools perception_reproducer.py -b <bag-file> -p
```

## Perception replayer

A part of the feature is under development.

This script can overlay the perception results from the rosbag on the planning simulator.

In detail, this script publishes the data at a certain timestamp from the rosbag.
The timestamp will increase according to the real time without any operation.
By using the GUI, you can modify the timestamp by pausing, changing the rate or going back into the past.

### How to use

First, launch the planning simulator, and put the ego pose.
Then, run the script according to the following command.

By designating a rosbag, perception replayer can be launched.
The GUI is launched as well with which a timestamp of rosbag can be managed.

```bash
ros2 run planning_debug_tools perception_replayer.py -b <bag-file>
```

You can designate multiple rosbags in the directory.

```bash
ros2 run planning_debug_tools perception_replayer.py -b <dir-to-bag-files>
```

Instead of publishing predicted objects, you can publish detected/tracked objects by designating `-d` or `-t`, respectively.

## Processing time checker

The purpose of the Processing Time Subscriber is to monitor and visualize the processing times of various ROS 2 topics in a system. By providing a real-time terminal-based visualization, users can easily confirm the processing time performance as in the picture below.

![processing_time_checker](image/processing_time_checker.png)

You can run the program by the following command.

```bash
ros2 run planning_debug_tools processing_time_checker.py -f <update-hz> -m <max-bar-time>
```

This program subscribes to ROS 2 topics that have a suffix of `processing_time_ms`.

The program allows users to customize two parameters via command-line arguments:

- --max_display_time (or -m): This sets the maximum display time in milliseconds. The default value is 150ms.
- --display_frequency (or -f): This sets the frequency at which the terminal UI updates. The default value is 5Hz.

By adjusting these parameters, users can tailor the display to their specific monitoring needs.

## Logging Level Updater

The purpose of the Logging Level Updater is to update the logging level of the planning modules via ROS 2 service. Users can easily update the logging level for debugging.

```bash
ros2 run planning_debug_tools update_logger_level.sh <module-name> <logger-level>
```

`<logger-level>` will be `DEBUG`, `INFO`, `WARN`, or `ERROR`.

![logging_level_updater](image/logging_level_updater.png)

When you have a typo of the planning module, the script will show the available modules.

![logging_level_updater_typo](image/logging_level_updater_typo.png)
