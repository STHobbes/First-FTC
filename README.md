# First-FTC

* status: In development, expect frequent changes.
* version: 0.65 - fixed power_accel_decel error in acceleration ramp; adjusted drive constants and motor directions.
* version: 0.6 - The full suite is here for mecanum drives. The other referenced drives will be added for release 1.0

These are some FTC robotics code samples for mentoring FTC teams at Hood River Valley High
School (HRVHS). The majority of our teams are programming in blocks, some in on-bot-java.
Java in android-studio is generally behind reach. However, samples here will be in all three
for completeness and to make comparison easy.

## Drive or Traction Implementation

At HRVHS we generally start with the construction of a robot base that can be driven around
before we start customizing for the specific competition. The base requires a physical
implementation, a software controller implementation, and software implementation of move
functions that can be used to build the autonomous program.

The selection of physical implementation from most simple is:
* **traction** tires: 4 tires with a fixed tread. Left side and right side may each be driven by a single motor or
  there may be one motor per wheel. Motion supported by this implementation is forward-backward and rotation. The
  center for rotation is the centroid of the drive wheels.
* **traction plus omni**: 2 traction tires, 2 omni tires. The omni tires may be either front or back. Omni tires reduce
  friction when turning, and move the center of rotation between the traction tires. The omni wheels have rollers
  mounted around their circumference that are coaxial with the plane of the wheel.
* **all omni**: 4 omni wheels when mounted at 45 degrees to the front of the robot support forward-backward, rotation,
  and sideways motion.
* **mecanum**: similar to omni, except the rollers are at 45 degrees to the axis of the wheels, so the wheels are all
  facing forward. Supports forward-backward, rotation, and sideways motion.
* **other**: *TODO: elaborate*

One secret to having a robot that performs well in competition is making it easy to drive for the tasks of the
competition. What is most easy to drive will depend on the physical drive implementation, the programming
implementation that connects the controller to the physical implementation, and the driver who may be most comfortable
with move paradigms that mimic familiar games.
 
**The control mode paradigm**

Most simple programming examples use *tank* mode where the right and left stick Y is directly mapped to the left and
right motor speed. This is by far the easiest to program, but it is not an intuitive interface unless you have
experience with tracked heavy equipment like tanks, bulldozers, etc. The most intuitive for the driver is often
a mode that most closely mimics the authors favorite games.

These are the control modes demonstrated in this code:
* **tank**: left stick controlling speed of left tires, right stick controlling speed of right tires. Tank is the
  most simple type to program, but is challenging to control. Simple things like straight forward motion is difficult
  at slow speeds.
* **airplane** where multiple move directions, i.e. forward-backward and
  either sideways or rotate are on the same stick.
* **auto** (do be added in the sample programs): where 1 stick controls direction (like the steering wheel
  of a car), and some other control (the other stick, or a trigger), controls speed (like an accelerator pedal).

**Controlling Sensitivity**

Robots need to get somewhere on the playing field quickly. Then they need to do something. I have often seen robots
getting to *about the right place* in seconds, and then taking 10's of seconds fumbling around because the have
extended an arm and need to move it left an inch, but any turn moves the arm 6" so a dozen moves are required
before the arm fortunately stops in the right place.

The problem has 2 primary causes:
* The control sticks are designed for *twitch game - fast extreme motion is more important than fine control and
  there is very little control distance between 0 and full speed.
* Loop speeds - the time between when you make a command and when the robot responds is potentially long and
  highly variable, which encourages over-control. Specifically, the driver asks for small motion, does not see
  immediate response, and then asks for more motion, and by the time the robot responds, the motion is way faster
  than desired
  
I have tried different functions like controller position squared or cubed to try to get more sensitivity around 0,
and have concluded that is the long loop time that causes the majority of fine control problems. The only reasonable
solution I have found is a *fine control mode* that limits the maximum value from the stick to something that achieves
fine control, and training drivers to use it when they need fine control - see **right bumper** in the next section.

**Using the Sample Drive Code**

In all of the example programs:
* **`B` button**: changes the drive mode.
* **`X` button**: terminates the program. You
  and your team are encouraged to try all of the drive modes and add additional ones for testing before you settle
  on the specific drive mode(s) you will use for your robot.
* **right bumper**: applies a `bumper_speed` multiplier to the stick values for fine control. You are encouraged to
  try different bumper speeds to find one that best helps you control the robot when you need fine control.
* **dpad**: the datapad controls forward-backward (up-down), sideways (right-left), and rotation (right-left with
  the left bumper pressed).

### Traction

*TODO*

### Traction plus Omni

*TODO*

### Omni Only

*TODO*

### Mecanum 

With mecanum wheels we have found that the motion of the robot changes when the balance of the robot changes. For
example, if the robot is very front heavy it will tend to rotate clockwise when moving right and counterclockwise
when moving left. Also, if the wheel mounting is not rigid (wheel axles supported on both ends), this will
affect movement. The robot will also be easier to displace (move or rotate) if struck by other robots than a robot
with traction wheels. The physical implementation this code was written for uses these
[mecanum wheels](https://www.amazon.com/100Mm-Aluminum-Mecanum-Wheel-Right/dp/B01CTUT4GY/ref=sr_1_1)
driven by [Rev core hex](http://www.revrobotics.com/rev-41-1300/) motors
 
These are the sample programs for mecanum wheels:
* **blocks/mecanum_drive_example.blk** - Running the mecanum wheels from blocks with all the controller options
  described above. While blocks are easy to program, it is the case that if there are multiple topologies for
  autonomous in addition to the TeleOp there will be multiple blocks programs for the competition. It is difficult
  to keep multiple blocks programs in sync when there are changes to the base robot.
* **on_bot_java/java/org/firstinspires/ftc/teamcode/** - programs that can be uploaded to OnBotJava
  * **MecanumDriveExampleOrig** - The OnBotJava version of the blocks program as it is originally
    translated from the blocks program.
  * **MecanumDriveExample** - Simplifying the above program by using some Java constructs that are
    not well supported in blocks.
  * **MecanumBase** - a base class extracted from the **MecanumDriveExample**. Building a base class for your robot
    lets you put all the common robot-specific code in one place so you can respond to changes in the robot by
    adjusting the base class.
    * **MecanumTest** - this is the **MecanumDriveExample** runOpMode implemented as an extension of **MecanumBase**
    * **MecanumTeleOp** - this is a runOpMode implemented as an extension of **MecanumBase** that would be the
      starting point for building the driver control program. It uses the airplane right driver control and does
      not support the other drive modes.
    * **MecanumAuto** - this is the **MecanumDriveExample** runOpMode implemented as an extension of **MecanumBase**
      that runs an autonomous test pattern, and could be easily customized for a specific game topology. NOTE: This
      autonomous pattern takes longer than the 30sec competition limit to run - so turn the timer off if you run this.
* **ftc_app/java/org/firstinspires/ftc/teamcode/**
  * *TODO*

## TeamCode Module

Read the FTC app documentation and readme.md for details of adding opcodes, building, etc. This is my code from
the 2018-2019 Rover Ruckus FTC season. In this year's code I tried to focus on a structure that makes it easy to
program and test robot components in isolation, and then easily assemble the components into robot calibration,
autonomous, and driver control.
 
I also wanted to use this structured approach for building code samples demonstrating different methods for
user control of components.

## The Robot Drive Chassis, or Traction

In an ideal world, the traction for the robot would be abstracted so the various calibration, autonomous, and
driver control programs are independent of changes to the physical implementation of the traction. In this code
there is an `ITraction` interface that represents the traction operations you want to perform in your programs.
Implementations of `ITraction` are specific to your physical drive chassis. The `RuckusDrive` class implements
`ITraction` for my mecanum drive chassis.

### The `ITraction` Interface

The traction interface needs to encapsulate the different properties of the drive or traction mechanism. The
drives that I have encountered always have forward/backward, and turn (or rotation). Note that turn becomes rotation
if the turning radius is 0. Some drives like omni or mecanum also support sideways motion.



