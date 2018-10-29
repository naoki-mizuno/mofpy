# mofpy

<div style="margin: 0 auto" >
    <img src="misc/logo.png" alt="Logo of mofpy" />
</div>

## What This Is

Mofpy is a ROS node that lets you to generically trigger actions from joypad
inputs. For example, you can publish certain message like a
`geometry_msgs/Twist` when pressing the `R1` button. Normally, you need to
write your own application that subscribes to the `joy` topic, check whether
the index corresponding to the `R1` button is high or low, and if low, publish
the desired message. There are two problems to this:

1. The index you need to check is different depending on what joypad you're
   using.
2. The node you wrote can only be used for the specific case when you want to
   publish a `geometry_msgs/Twist`.

Mofpy attempts to solve these issues by allowing you to create virtual axes
and buttons (by giving them names like `R1`, `O`, `X`) and mapping them to
certain actions (such as publishing a twist, planning and executing to a
move_group state, etc).

These are the benefits of using mofpy:

- allows you to reuse the joypad configurations that you previously wrote
- saves you from rewriting similar subscribe-to-joy-and-publish-something
  nodes
- allows you to chain actions, making it possible to bind macro-style
  actions to joypad inputs

## Supported Joypads

The following controllers are supported by default.

- PS4 (Wired): `ps4_wired.yaml`
- PS4 (Wireless): `ps4_wireless.yaml`
- Elecom JC-U3412S: `elecom_small.yaml`
- Elecom JC-U4113S: `elecom_large.yaml` (Coming soon)

## Supported Actions

- Publish 6DOF `cmd_vel` for manipulators
- Publish 2DOF `cmd_vel` for ground vehicles
- Publish any message of constant value
- Plan and execute to a predefined move_group state
- Sleep for a certain duration (useful for waiting between actions)

## Examples

The following examples demonstrate what you can do with mofpy.

- `single_press`: When pressing `R1`, publish the value 42 to the `foo` topic
  of type `std_msgs/Float64`.
- `multiple_press`: When sequentially pressing `R1`, `L1`, `R1`, `L1`, publish
  the string `Hello, World!` to the `hello` topic.
- `long_press`: When pressing `R1` for 1 second, publish a
  `geometry_msgs/PointStamped` message to the `fake_point` topic with the
  given values. The `header` field is populated by filling in the current
  time for `stamp` and `frame_id` set to empty.
- `multiple_long_press`: When pressing `R1` and `L1` simultaneously for 1
  second, publish a `geometry_msgs/Pose`, then sleep for 3 seconds, and then
  publish a `sensor_msgs/Imu` message. The `stamp` field in this message will
  be substituted by the current time.

```yaml
presets:
  single_press:
    # Single short press
    trigger: R1
    action:
      - type: publish
        topic:
          name: foo
          type: std_msgs/Float64
        values:
          data: 42
  multiple_press:
    # Sequential button press
    trigger: [R1, L1, R1, L1]
    action:
      - type: publish
        topic:
          name: hello
          type: std_msgs/String
        values:
          data: 'Hello, World!'
  long_press:
    # Long press for 1 second
    trigger: [R1, 1]
    action:
      - type: publish
        topic:
          name: fake_point
          type: geometry_msgs/PointStamped
        values:
          header: auto
          point:
            x: 1
            y: 2
            z: 3
  multiple_long_press:
    # Long press R1 and L1 simultaneously for 1 second
    trigger: [[R1, L1], 1]
    action:
      - type: publish
        topic:
          name: fake_pose
          type: geometry_msgs/Pose
        values:
          position:
            # Note: Use the default values for the other fields
            x: 2
          orientation:
            w: 1
      - type: sleep
        duration: 3
      - type: publish
        topic:
          name: fake_imu
          type: sensor_msgs/Imu
        values:
          header:
            stamp: now
            frame_id: imu
          orientation:
            w: 1
          linear_acceleration:
            z: 9.8
```

In `multiple_long_press`, pressing `R1` and `L1` simultaneously for  1 second
will publish the value `42` to `foo` topic, immediately followed by another
message published to the `foo` topic with the value `43`.

## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
