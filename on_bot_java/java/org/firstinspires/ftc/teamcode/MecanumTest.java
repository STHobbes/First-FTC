package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 
 */
@TeleOp(name = "MecanumTest", group = "")
public class MecanumTest extends MecanumBase {

  // The calibration distance, which is the distance the robot will move
  // for dpad forward, backward, right, and left.
  double calibration_distance = 24.0;
  // A turning rate when in automotive drive mode to limit the turn rate a full
  // forward or backward speed.
  double auto_turn_rate = 0.25;

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {
    
    int drive_mode;
    int last_drive_mode;
    String[] drive_modes = {"tank", "airplane right", "airplane left",
        "auto right", "auto left"
    };
    
    // initialize the mecanum base
    initialize_pre_start(hardwareMap); 
    // initialize drive mode
    drive_mode = 0;
    last_drive_mode = 0;
    
    waitForStart();
    initialize_post_start();
    
    // This is the main run loop, we start in tank mode, but use the B button
    // to switch to airplane, use X to exit
    while (opModeIsActive()) {
      if (gamepad1.x) {
        // Test for finished
        break;
      } else if (gamepad1.b) {
        // Test for switch between tank and aircraft
        if (drive_mode == last_drive_mode) {
          drive_mode = (drive_mode + 1) % drive_modes.length;
        }
      } else if (gamepad1.dpad_up) {
        // Move forward the calibration distance
        move(calibration_distance, 0.0);
      } else if (gamepad1.dpad_down) {
        // Move backwards the calibration distance
        move(-calibration_distance, 0.0);
      } else if (gamepad1.dpad_right) {
        if (gamepad1.left_bumper) {
          // Rotate 90 clockwise
          rotate(90.0);
        } else {
          // Move right the calibration distance
          move(calibration_distance, 90.0);
        }
      } else if (gamepad1.dpad_left) {
        if (gamepad1.left_bumper) {
          // Rotate 90 counter-clockwise
          rotate(-90.0);
        } else {
          // Move left the calibration distance
          move(calibration_distance, -90.0);
        }
      } else if (gamepad1.y) {
        // move in a 30 square rotated 30 degrees
          move(12.0, 30.0);
          move(-12.0, -60.0);
          move(-12.0, 30.0);
          move(12.0, -60.0);
      } else {
        last_drive_mode = drive_mode;
        telemetry.addData("drive mode", drive_modes[drive_mode]);
        // This is the real drive loop
        // Condition the X-Y stick inputs
        condition_sticks();
        // Set the motor speeds
        if (drive_mode == 0) {
          // Tank drive - motor speeds are stick Y for both left and right
          tank_drive();
        } else if (drive_mode == 1) {
          // right stick is forward/back and side-to-side with no rotation,
          // left X is rotation.
          set_speeds(right_y, right_x, left_x);
        } else if (drive_mode == 2) {
          // left stick is forward/back and side-to-side with no rotation,
          // right X is rotation.
          set_speeds(left_y, left_x, right_x);
        } else if (drive_mode == 3) {
          set_speeds(right_y, left_x,
            right_x + ((right_x * Math.abs(right_y)) * (auto_turn_rate - 1.0)));
        } else {
          set_speeds(left_y, right_x,
            left_x + ((left_x * Math.abs(left_y)) * (auto_turn_rate - 1.0)));
        }
      }
      telemetry.update();
    }
  }
}
