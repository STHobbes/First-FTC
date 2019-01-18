# First-FTC

* status: Release
* version: 0.90 - All the basic programming principles for tractions are here.
* TODO: The final versions are all in the android programming directory (ftc-app). Not all of these have been
  back-ported to blocks and on-bot-java (I will get to that).

These are some FTC robotics code samples for mentoring FTC teams at Hood River Valley High
School (HRVHS). The majority of our teams are programming in blocks, some in on-bot-java.
Java in android-studio is generally behind reach. However, samples here will be in all three
for completeness and to make comparison easy.

## Drive or Traction Implementation

At HRVHS we generally start with the construction of a robot base (a simple *squarebot* - 4 traction
tire; 12" to 18" square base) that can be programmed and driven around
before we start customizing for the specific competition. We call the drive base the **traction** for the
robot. The **traction** requires a physical implementation (frame,
motors, wheels), a software controller configuration (connection of the REV controller to the motors, IMU, and
other sensors), and software implementation of move functions that can
be used to drive the robot and build the autonomous
program. This project is all about the software implementation of driver control and move functions.

The selection of physical implementation from most simple is:
* **traction tires**: 4 traction tires with a fixed tread - this is the *squarebot*. Left side and right side may
  each be driven by a single motor or there may be one motor per wheel. Motion supported by this implementation is
  forward-backward and rotation. The center for rotation is the centroid of the drive wheels.
* **traction plus omni**: 2 traction tires, 2 omni tires. The omni tires may be either front or back. Omni tires reduce
  friction when turning, and move the center of rotation between the traction tires. The omni wheels have rollers
  mounted around their circumference that are coaxial with the plane of the wheel.
* **all omni**: 4 omni wheels when mounted at 45 degrees to the front of the robot support forward-backward, rotation,
  and sideways motion.
* **mecanum**: similar to omni, except the rollers are at 45 degrees to the axis of the wheels, so the wheels are all
  facing forward. Supports forward-backward, rotation, and sideways motion.
* **other**: *TODO: elaborate*

One secret to having a robot that performs well in competition is making it easy to drive for the tasks of the
competition, and easy to reliably program the autonomous operation of the robot.

## General Programming Notes

I'm trying to build example code for each of the traction types listed above. The objective is to abstract
the operation of the traction to a couple simple methods supporting different drive paradigms, and
a couple simple methods that can be used for building the autonomous OpMode; all described later in
this section.

### Organization

This repository is organized into 3 main areas:
* **blocks** - Contains the blocks code downloaded from the blocks programming environment, which can be
  uploaded to your blocks programming environment. There is (or will be) a
  blocks program for each of the tractions that handles the function of the test program described in **on_bot_java**.
  The blocks program will have a name like *traction*_drive_example.blk where *traction* is the type of traction, i.e.
  the blocks program for mecanum wheels will have a name like `mecanum_drive_example.blk`
* **on_bot_java/java/org/firstinspires/ftc/teamcode/** - The Java code downloaded from the OnBotJava programming
  environment, which can be uploaded
  to your OnBotJava programming environment. For each traction, there will be 2 programs that come directly from
  the blocks program:
  * ***Traction*DriveExampleOrig.java** - The blocks file as it was automatically translated to Java by the block
    programming environment with minor problems (usually variable types) fixed.
  * ***Traction*DriveExample.java** - A cleanup of *Traction*DriveExampleOrig.java to follow better Java
    programming practices.
    
  Then there will be a more Java approach that pulls the common drive functionality into a base class
  * ***Traction*Base.java** - a base class for the *traction*.
  
  Then there will test and autonomous programs that can be run using any of the *Traction*Base.java classes as it's
  base class (a one line change), all currently supported *traction*s are included in commented code.
  * **TractionTest** - Implements a number of different driver control paradigms to help you in determining which
    is the best match for your competition and driver. Implements buttons that run autonomous move and rotate
    functions to help you calibrate and tune the variables that control your traction.
  * **TractionAuto** - A simple autonomous program that should move your robot along the same path regardless
    of the traction you use.
* **ftc_app/java/org/firstinspires/ftc/teamcode/** - focuses on demonstrating how you can make the drive a
  component using an interface. I find this approach more flexible as a class can implement many interfaces while
  class can only extend a single class. This is specifically useful when I have multiple robots (both a prototype and
  a competition robot) and I want to work on and test my competition programming using either robot. The key here is
  that we define an interface to the drive, and a class implementing that interface for any physical drive (a traction
  component for that drive). In the competition or test programs we simply load the correct traction component for
  the robot, and the robot should do the same thing regardless of the base.
  * **ITraction.java** - The interface for a traction component. This is simplified from the **on_bot_java** in that there
    are minimal methods: `initialize()` and `postStartInitialize()` for traction initialization; `supportsSideways()`
    for capability; `setSpeeds()` and `setTankSpeeds()` for setting motor speeds in ether driver or autonomous
    control; and `move()` and `rotate()` and autonomous traction operations.
  * **TractionBase.java** - A base implementation that has the acceleration-deceleration ramp method, and default
    implementations of `supportsSideways()` and `move()` for a simple traction that supports forward-backward
    motion and not sideways.
    
  Then there will test and autonomous programs that can be run using any of the ITraction.java implementations as
  the traction is loaded at initialization (a one line change), all currently supported ITraction
  implementations are included in commented code.
  * **TractionTest** - Implements a number of different driver control paradigms to help you in determining which
    is the best match for your competition and driver. Implements buttons that run autonomous move and rotate
    functions to help you calibrate and tune the variables that control your traction.
  * **TractionAuto** - A simple autonomous program that should move your robot along the same path regardless
    of the traction you use.

### The control mode paradigm

Most simple programming examples use *tank* mode where the right and left stick Y are directly mapped to the left and
right motor speeds. This is by far the easiest to program, but it is not an intuitive interface unless you have
experience with tracked heavy equipment like tanks, bulldozers, etc. The most intuitive for the driver is often
a mode that most closely mimics the driver's favorite games or vehicles.

These are the control modes demonstrated in this code:
* **tank**: left stick controlling speed of left tires, right stick controlling speed of right tires. Tank is the
  most simple type to program, but is challenging to control. Simple things like straight forward motion is difficult
  at slow speeds.
* **airplane right/left** where multiple move directions, i.e. forward-backward and
  either sideways or rotate are on the same stick.
* **auto right/left**: where 1 stick-x controls direction (like the steering wheel
  of a car), and some other control (the stick-y, or a trigger), controls speed (like an accelerator pedal).
  
### Using the Sample Drive Program

In all of the example programs there is a test drive program with these controls:
* **`B` button**: changes the drive mode. The modes are **tank**, **airplane right**, **airplane left**,
    **auto right**, and **auto lef**. See earlier notes for details.
* **`X` button**: terminates the program. You
  and your team are encouraged to try all of the drive modes and add additional ones for testing before you settle
  on the specific drive mode(s) you will use for your robot.
* **`Y` button**: The robot moves in a square cocked 30 degrees clockwise.
* **right bumper**: applies a `bumper_speed` multiplier to the stick values for fine control. You are encouraged to
  try different bumper speeds to find one that best helps you control the robot when you need fine control.
* **dpad**: the datapad runs autonomous move operations:
  * up-down: the robot moves forward-backward the `calibration_distance`;
  * right-left: the robot moves sideways (right or left) the `calibration_distance`;
  * right-left with the left bumper pressed: the robot rotates 90&deg; clockwise or counter-clockwise.

### Controlling Sensitivity

Robots need to get somewhere on the playing field quickly. Then they need to do something. I have often seen robots
getting to *about the right place* in seconds, and then taking 10's of seconds fumbling around because they have
extended an arm and need to move it left an inch, but any turn moves the arm 6" so a dozen moves are required
before the arm fortunately stops in the right place.

The problem has 2 primary causes:
* The control sticks are designed for *twitch* game - fast extreme motion is more important than fine control and
  there is very little control distance between 0 and full speed.
* Loop speeds - the time between when you make a command and when the robot responds is potentially long and
  highly variable (i.e. the drive program is a loop and there is a lag between when the driver takes an action
  and the program/robot responds to the action because of loop time), which encourages over-control. Specifically,
  the driver asks for small motion, does not see immediate response, and then asks for more motion, and by the
  time the robot responds, the motion is way faster and more extreme than desired.
  
I have tried different functions like controller position squared or cubed to try to get more sensitivity around 0,
and have concluded that it is the long loop time that causes the majority of fine control problems. The only reasonable
solution I have found is a *fine control mode* that limits the maximum value from the stick to something that achieves
fine control, and training drivers to use it when they need fine control - see **right bumper** in the next section.

## Traction Wheels Only
Theses are programming notes for the simple *SquareBot* described earlier. For this exercise I built a small
*SquareBot* using REV motors and traction wheels.

*SquareBot*-specific programs are :
* **blocks/squarebot_drive_example.blk**
* **on_bot_java/java/org/firstinspires/ftc/teamcode/**
  * **SquareBotDriveExampleOrig.java**
  * **SquareBotDriveExample.java**
  * **SquareBotBase.java** - a base class for the *SquareBot*.
* **ftc_app/java/org/firstinspires/ftc/teamcode/**
    * **TractionBase** extensions:
      * **SquareBotTraction** - The implementation for the **SquareBot** using encoders and motor speed.
      * **SquareBotPidTraction** - The implementation for the **SquareBot**  using encoders, motor speed,
        and a PID loop to adjust heading.
## Traction plus Omni Wheels

*To be completed spring 2019*

## Omni Wheels Only

*To be completed spring 2019*

## Mecanum Wheels

With mecanum wheels we have found that the motion of the robot changes when the balance of the robot changes. For
example, if the robot is very front heavy it will tend to rotate clockwise when moving right and counterclockwise
when moving left. Also, if the wheel mounting is not rigid (wheel axles supported on both ends), this will
affect movement. The robot will also be easier to displace (move or rotate) if struck by other robots than a robot
with traction wheels. The physical implementation this code was written for uses these
[mecanum wheels](https://www.amazon.com/100Mm-Aluminum-Mecanum-Wheel-Right/dp/B01CTUT4GY/ref=sr_1_1)
driven by [Rev core hex](http://www.revrobotics.com/rev-41-1300/) motors
 
Mecanum-specific programs are:
* **blocks/mecanum_drive_example.blk**
* **on_bot_java/java/org/firstinspires/ftc/teamcode/**:
  * **MecanumDriveExampleOrig.java**
  * **MecanumDriveExample.java**
  * **MecanumBase** - a base for the mecanum wheels
* **ftc_app/java/org/firstinspires/ftc/teamcode/**
    * **TractionBase** extensions:
      * **MecanumTraction** - The implementation for the mecanum physical traction using encoders and motor speed.
      * **MecanumPidTraction** - The implementation for the mecanum physical traction using encoders, motor speed,
        and a PID loop to adjust heading.

