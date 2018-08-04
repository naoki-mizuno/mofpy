# mofpy

Generic-joypad-input-to-whatever node, where whatever can be, but not limited
to:

- publishing twist messages for a 6DOF manipulator
- moving the manipulator to a certain state using the move_group commander
- publishing values to topics
- ...and whatever you define!


## Description

TODO


## Defining the Axes and Buttons

The first thing you need to do is give each physical axes and buttons a name.
You only need to do this for the ones that you are going to use, but it is
recommended that you name all your axes and buttons.

### Normal Case

Normally, you would use an axis as an axis and a button as a button:

```yaml
virtual:
  axes:
    AXIS_NAME:
      real:
        type: axis
        index: 0
  buttons:
    BUTTON_NAME:
      real:
        type: button
        index: 0
```

`AXIS_NAME` is the name of the virtual axis/button, which is bound to the
physical axis 0. Similarly, `BUTTON_NAME` is the name given to button 0.

### Special Case

In special cases, sometimes you want to use an axis as a button or the other
way around.

```yaml
virtual:
  axes:
    PHYSICAL_BUTTON:
      real:
        type: button
        index: 1
  buttons:
    PHYSICAL_AXIS:
      real:
        type: axis
        index: 1
      low_range: [0, 0.5]
      high_range: [0.5, 1]
```

`PHYSICAL_BUTTON` is an example of using a button as axis. When the button is
pressed, the axis value is 1.0, otherwise 0.0. `PHYSICAL_AXIS` is using an
axis as a button. If the axis value is within the `low_range`, it is low (i.e.
considered unpressed) and if it's within `high_range`, it is high.


## Using the Bundled Presets

Once you've given names to the axes and buttons, you can use those names to
trigger certain presets. A "preset," which is "triggered" by a "command",
consists of multiple "actions." Following are some simple preset definitions
that publishes a `Float64` message to the `foo` topic:

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


## Defining a Preset

1. Create a file in `src/mofpy/action/`
2. Define a class that inherits from `Action`
3. Define a class variable named `NAME` containing the name of the preset
4. Define the `__init__` method which takes a dictionary as an argument
5. Override the `execute` method
6. Add `Action.register_preset(YOUR CLASS)` at the end of the file

Here's an example:

```python
from .action import Action


class Foobar(Action):
    NAME = 'foobar'

    def __init__(self, definition):
        super(Foobar, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        # You can retrieve parameters
        self.__my_param = self.get_required('my_param')

    def execute(self, named_joy=None):
        print('foobar called! my_param == {0}'.format(self.__my_param))


Action.register_preset(Foobar)
```

This action can be used like this:

```yaml
presets:
  trigger_foobar:
    trigger: [O, X]
    action:
      - type: foobar
        my_param: 42
```

When `O` followed by a `X` is pressed, this will trigger the `trigger_foobar`
preset, which executes the `foobar` action. Note that `my_param` must be
defined otherwise it will raise an exception.


## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
