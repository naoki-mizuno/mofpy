# mofpy

<div style="margin: 0 auto" >
    <img src="misc/logo.png" alt="Logo of mofpy" />
</div>

## What This Is

mofpy is a ROS node that lets you convert joypad input to actions. Normally,
you need to write your own application to achieve this conversion.

## Supported Joypads

The following controllers are supported by default.

- PS4 (Wired): `ps4_wired.yaml`
- PS4 (Wireless): `ps4_wireless.yaml`
- Elecom JC-U3412S: `elecom_small.yaml`
- Elecom JC-U4113S: `elecom_large.yaml` (Coming soon)

## Supported Actions

- Publish 6DOF `cmd_vel` for manipulators
- Publish 2DOF `cmd_vel` for ground vehicles
- Call a move_group state
- TODO: Comprehensive list of the actions

## Examples

```yaml
presets:
  single_press:
    # Single short press
    trigger: R1
    action:
      - type: publish_float64
        topic: foo
        value: 42
  multiple_press:
    # Sequential button press
    trigger: [R1, L1, R1, L1]
    action:
      - type: publish_float64
        topic: foo
        value: 42
  long_press:
    # Long press for 1 second
    trigger: [R1, 1]
    action:
      - type: publish_float64
        topic: foo
        value: 42
  multiple_long_press:
    # Long press R1 and L1 simultaneously for 1 second
    trigger: [[R1, L1], 1]
    action:
      - type: publish_float64
        topic: foo
        value: 42
      - type: publish_float64
        topic: foo
        value: 43
```

In `multiple_long_press`, pressing `R1` and `L1` simultaneously for  1 second
will publish the value `42` to `foo` topic, immediately followed by another
message published to the `foo` topic with the value `43`.

## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
