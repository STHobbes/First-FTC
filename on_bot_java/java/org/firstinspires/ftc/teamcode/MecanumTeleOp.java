package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This class extends the base for driver control. This extension includes
 * a number of different mappings to the gamepad control sticks. In your
 * implementation you may want to implement only the mapping most natural
 * to the competition at hand.
 */
@TeleOp(name = "MecanumTeleOp", group = "")
public class MecanumTeleOp extends MecanumBase {

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {
    // initialize the mecanum base
    initialize_pre_start(hardwareMap); 
    waitForStart();
    initialize_post_start();
    
    // This is the main run loop, we start in tank mode, but use the B button
    // to switch to airplane, use X to exit
    while (opModeIsActive()) {
      if (gamepad1.x) {
        // Test for finished
        break;
      } else {
        // Condition the X-Y stick inputs
        condition_sticks();
        // right stick is forward/back and side-to-side with no rotation,
        // left X is rotation.
        set_speeds(right_y, right_x, left_x);
      }
      telemetry.update();
    }
  }

}
