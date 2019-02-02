[home](./README.md) - Mecanum Traction Details

# Mecanum Traction Details

A Mecanum traction is only cool when you can make it work intuitively for the driver. While Mecanum wheels are
exceptionally cool, they are fraught with weird behaviour, especially as robot balance and dynamics
change during competition (when arms are stretched out and carrying loads, etc.). This is a primer on Mecanum
wheels and what we at HRVHS robotics have learned in working with them:
* You need to understand them to program for them;
* Tank drive is not intuitive or easy for the driver;
* Mapping control stick values to motor speeds must be done carefully  - clipping speeds to the range (-1, 1)
  gives really undesirable results.
* The behaviour (i.e. the tracking) of the robot changes dramatically when the balance of the robot changes. For
  example, if the robot is very front heavy it will tend to rotate clockwise when moving right and counterclockwise
  when moving left.
  
The big challenges are making driver control programs intuitive, and autonomous programs predictable and
reliable. When discussing this with one of our programmers, they said *the driver can just learn and adjust
to what we give them*. People can learn to do [this](https://www.youtube.com/watch?v=b4RAx9BWc-w), but asking
your driver to learn something of this complexity for competition is a recipe for failure - *make it easy and
intuitive for your driver* - that way they can focus on performing well for the competition rather than focusing
all their attention on just getting the robot to move on the field.

The physical implementation this code was written for uses these
[mecanum wheels](https://www.amazon.com/100Mm-Aluminum-Mecanum-Wheel-Right/dp/B01CTUT4GY/ref=sr_1_1)
driven by [Rev core hex](http://www.revrobotics.com/rev-41-1300/) motors.

## Understanding Mecanum Wheels

[AndyMark](https://www.andymark.com/) is one of our sources for FTC/FRC components and they have this excellent
[Mecanum Wheel Spec Sheet](http://files.andymark.com/MecanumWheelSpecSheet.pdf) (cached
[here](../resources/MecanumWheelSpecSheet.pdf) in case the link is broken). These are the things to take away
from this data sheet:
* **There are 2 different wheel types in a set, and they need to be mounted in the right configuration** -***PAY
  ATTENTION TO THIS!!*** I can't tell you how many times our teams get this wrong, and I have gotten this wrong. The
  wheels don't work if you get this wrong.
* **The primary force of the wheels is at 45&deg; to the front-back axis of the robot** - this means that friction
  is a primary consideration in making these wheels work. If the robot is not well balanced, or the playing field
  is not flat and uniform friction everywhere, the friction is not
  uniform for all of the wheels - so the wheels don't work as expected. We need to fix this in the programming.
  
OK, the first bullet just means we need to mount the wheels correctly; the second means we need to program well
for the driver to be able to realize the full potential of the robot. So let's get the starting point for
programming. Let's start with these inputs:
* **forward** - in the range -1 (full speed backwards) to 1 (full speed forwards); 
* **sideways** - in the range -1 (full speed left) to 1 (full speed right); 
* **rotate** - in the range -1 (full speed counter-clockwise) to 1 (full speed clockwise).

So let's interpret the data sheet and compute the relative motor speeds based on the data sheet. Let's assume
you have configured `power_rf`, `power_rr`, `power_lf`, and `power_lr` to refer to the speed (since
these motors are configured
for `RUN_USING_ENCODER` mode); referring to the right-front, right-rear, left-front, and left-rear motors
respectively. So these are the motor speeds based on the spec sheet:

```$xslt
power_rf = forward - sideways - rotate
power_rr = forward + sideways - rotate
power_lf = forward + sideways + rotate
power_lr = forward - sideways + rotate
```

Note that in driver mode we are getting the `forward`, `sideways`, and `rotate` values from multiple sticks, and that
for even a single stick, the sum of the stick values for these components can be greater that 1 or less
than -1. This is a problem.

Clipping an arbitrary number of motor speed values creates an imbalance that does not move the robot in the requested
direction. In the `set_speeds` method, the first thing we do is compute a scale that keeps all motor speeds in the 
-1 to 1 range without compromising the relative ratios of `forward`, `sideways`, and `rotate` - which means the robot
will move as we expect:

```$xslt
scale = 1.0
max = abs(forward) + abs(sideways) + abs(rotate)
if (max > 1.0) {
    scale = 1.0 / max
}
power_rf = scale * (forward - sideways - rotate)
power_rr = scale * (forward + sideways - rotate)
power_lf = scale * (forward + sideways + rotate)
power_lr = scale * (forward - sideways + rotate)
```

If you add this check to scale then you will solve many of the unpredictable behaviour problems
experienced at extreme stick positions.

## Tank Drive

Simply moving forward in a straight line at anything other than full speed line using tank drive is nearly
impossible for all of our HRVHS drivers. Yes, one can learn to do this
if they practice long enough, but generally our drivers don't have years of operating tanks or
heavy earth-moving equipment to fall back on - so becoming proficient in tank drive is a big ask.

This is the easiest to program, but, it does still have the problem that sideways motion is combined with
forward-backward, and the sum can be greater then 1 or less than -1, so the scaling described in the previous
section needs to be applied.

## Mapping Joysticks to Forward, Sideways, Rotate

There are many options for this. The example program provides 5 different mappings from control sticks to
robot motion. Our students find airplane-right or airplane-left to be most natural (depending on video games
they are most familiar with). In this mapping, one stick controls forward-backwards-sideways with no change
in robot heading, and the other stick controls rotation.

## Does the Robot Follow the Command - PID control

  

