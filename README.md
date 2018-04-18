# mofpy

Convert a Joy command to Twist. And more.


## Description

The package converts inputs from a joypad to a 6DOF velocity command (Twist)
using a predefined mapping. It was developed to control the end-effector of a
manipulator, but it can be used for any application that requires a 6DOF
control.

It also has a feature where presets (which is a set of tasks) can be executed
on certain events such as long-pressing, triple-pressing, etc. some buttons.


## Mapping for Twist

The mapping is done using ROS parameters, so they can be written in a YAML
file. Let's jump into examples:

```yaml
mapping:
  x: a1
  y: a0
  z: [a5al7, a2al6]
  roll: -a3
  pitch: a4
  yaw: [b4, b5]
```


### The basics

The basic syntax is to just write the joypad axis that controls a certain
axis. `x: a1` indicates that axis 1 (which is the vertical movement of the
left stick) controls the `x` output. Same is for `y: a0`.


### Inverted axes

To express inverted axes, use the `-` sign. `roll: -a3` says that the roll
(`twist.angular.x`) is set to -1 times the value of axis 3.


### Buttons

Skipping to the last line, `yaw: [b4, b5]` indicates that button 4 is used to
control the plus direction of yaw (`twist.angular.z`) and button 5 for the
minus direction. When pressed, either a yaw value of 1 or -1 is set in the
twist.


### Active-low

This is where it gets a bit tricky. Some axes are active-low, meaning that
they are high (1.0) when NOT pressed and low (-1.0) when pressed. To indicate
active-low axes, use `aXal`, where `X` is the axis number.

However, there is a problem where active-low axes output 0 before being
pressed for the first time (rather than outputting -1.0). The only way to tell
whether an active-low axis has been pressed for the first time is to monitor
the buttons, because the button counterpart should be the inverse of the axis
(that is, the button outputs 0 when an active-low axis is non-zero, and
outputs 1 when the axis is 0). To add a button counterpart, use `aXalY`, where
`X` is the axis number and `Y` is the button number.


## Defining and Mapping Presets

The mapping of presets is done in two parts: the definition of the preset and
the mapping of that preset to a button event. Take a look at the following
example:

```yaml
mapping:
  presets:
    foo:
      event: double_press
      index: 3

presets:
  foo:
    - type: sleep
      duration: 1.5
    - type: move_group_state
      state_name: my_joint_pos
```

`event` can be one of the following: `button_down`, `button_up`,
`double_press`, `triple_press`, `long_press`. `long_press` requires an
additional parameter `duration`. `index` is the zero-based button index.

In the `presets` parameter, we specify the tasks to be executed when the
preset is called. In the example above, `foo` consists of a `sleep` and
`move_group_state` task. `sleep` is pretty self-explanatory.
`move_group_state` uses `move_group` to move to a named joint state (they are
defined in the srdf file in moveit config).

The available types are:

- `move_group_state`: Moves to a named joint position.
  - `state_name`: The name of the predefined joint position.
- `remember_move_group_state`: Remembers the current joint position in the
  - `state_name`: The name of the predefined joint position.
- `sleep`: Uses `rospy.sleep` to sleep for a certain duration.
  - `duration`: How long to sleep for.
  given name.
- `single_point_trajectory`: Publishes a JointTrajectory topic containing one
  point.
  - `topic`: Topic to publish to
  - `joints`: A dictionary of the joint names and their desired values.
  - `frame_id`: The frame ID of the message.
  - `execution_time`: How much time to spend on the execution. The value is
    used for the `time_from_start` field in the JointTrajectoryPoint message.


## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
