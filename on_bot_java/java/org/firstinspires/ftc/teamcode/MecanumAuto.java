package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Run an autonomous pattern.
 */
@Autonomous(name = "MecanumAuto", group = "")
public class MecanumAuto extends MecanumBase {

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() throws java.lang.InterruptedException {
    // initialize the mecanum base
    initialize_pre_start(hardwareMap); 

    waitForStart();
    initialize_post_start();
    
    // run an autonomous pattern
    forward(36.0);
    sideways(36.0);
    forward(-36.0);
    sideways(-36.0);
    Thread.sleep(2000);
    
    rotate(45.0);
    forward(50.9);
    rotate(225.0);
    forward(36.0);
    rotate(225.0);
    forward(50.9);
    rotate(-225.0);
    forward(36.0);
    rotate(90.0);
    Thread.sleep(2000);
    
    rotate(360.0);
    rotate(-360.0);
    
  }
}
