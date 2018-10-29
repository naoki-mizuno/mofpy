# Customizing

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


## Adding a Joypad

If you have a joypad that is not one of the bundled model, you can add your
own. You do this by giving a name for each axis or button that you want to
use.

### Use Axes/Buttons as is

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

### Use an Axis as a Button or Vice Versa

In special cases, sometimes you want to use an axis as a button or the other
way around.

```yaml
virtual:
  axes:
    NAME_OF_VIRT_AXIS:
      real:
        type: button
        index: 1
  buttons:
    DIRECTIONAL_UP:
      real:
        type: axis
        index: 1
      low_range: [0, 0.5]
      high_range: [0.5, 1]
    DIRECTIONAL_DOWN:
      real:
        type: axis
        index: 1
      low_range: [-0.5, 0]
      high_range: [-1.0, -0.5]
```

The first example uses the button with index 1 as an axis. The virtual axis is
named `NAME_OF_VIRT_AXIS`. The value of the axis is `1.0` when the button is
pressed and `0.0` when released.

The second example uses an axis as a button. A situation where this is useful
is when you want to use the directional buttons on your controller because
directional buttons are normally treated as axes, even though you press it
like buttons. In the example, a directional button is split into two buttons,
namely, `DIRECTIONAL_UP` and `DIRECTIONAL_DOWN`. `DIRECTIONAL_UP` is inactive when
the value is between 0 and 0.5, and is active when the directional button's
value is between 0.5 and 1.

Take a look at the bundled configurations to get an idea of how the virtual
buttons are defined.
