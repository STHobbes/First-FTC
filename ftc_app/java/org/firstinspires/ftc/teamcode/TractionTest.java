package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is a simple test program that exercises different drive controller options that will help you manually test
 * various driver interfaces for your traction, and various autonomous drive operations that will help you calibrate
 * your encoder tics-per-inch, acceleration and deceleration windows, etc. See the notes in README.md for
 * documentation of the control operations.
 */
@TeleOp(name = "TractionTest", group = "")
public class TractionTest extends LinearOpMode {
    private int drive_mode = 0;
    private int last_drive_mode = drive_mode;
    private String[] drive_mode_name = {"tank", "airplane right", "airplane left",
            "auto right", "auto left"};
    // Load the implementation of the program for your physical implementation of the traction here.
//    private ITraction traction = new MecanumTraction();         // Mecanum, no PID heading correction
    private ITraction traction = new MecanumPidTraction();      // Mecanum, with PID heading correction
//    private ITraction traction = new SquareBotTraction();       // SquareBot, no PID heading correction
//    private ITraction traction = new SquareBotPidTraction();    // SquareBot, no PID heading correction

    private double bumper_speed = 0.6;
    // The calibration distance, which is the distance the robot will move
    // for dpad forward, backward, right, and left.
    double calibration_distance = 24.0;
    // gamepad state
    private double left_x = 0.0;
    private double left_y = 0.0;
    private double right_x = 0.0;
    private double right_y = 0.0;

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        // initialize the mecanum traction base.
        traction.initialize(this);
        this.waitForStart();
        traction.postStartInitialize();
        while (this.opModeIsActive()) {
            if (gamepad1.x) {
                // Test for finished
                break;
            } else if (gamepad1.b) {
                // Test for switch between tank and aircraft
                if (drive_mode == last_drive_mode) {
                    drive_mode = (drive_mode + 1) % drive_mode_name.length;
                }
            } else if (gamepad1.y) {
                // move in a 30 square rotated 30 degrees
                traction.move(12.0,30.0,1.0);
                traction.move(-12.0,-60.0,1.0);
                traction.move(-12.0,30.0,1.0);
                traction.move(12.0,-60.0,1.0);
            } else if (gamepad1.dpad_up) {
                // Move forward the calibration distance
                traction.move(calibration_distance,0.0,1.0);
            } else if (gamepad1.dpad_down) {
                // Move backwards the calibration distance
//                traction.move(calibration_distance,180.0,1.0);
                traction.move(-calibration_distance,0.0,1.0);
            } else if (gamepad1.dpad_right) {
                if (gamepad1.left_bumper) {
                    // Rotate 90 clockwise
                    traction.rotate(90.0, 1.0);
                } else {
                    // Move right the calibration distance
                    traction.move(calibration_distance,90.0,1.0);
                }
            } else if (gamepad1.dpad_left) {
                if (gamepad1.left_bumper) {
                    // Rotate 90 counter-clockwise
                    traction.rotate(-90.0, 1.0);
                } else {
                    // Move left the calibration distance
//                    traction.move(calibration_distance,-90.0,1.0);
                    traction.move(-calibration_distance,90.0,1.0);
                }
            } else {
                last_drive_mode = drive_mode;
                telemetry.addData("drive mode", drive_mode_name[drive_mode]);
                // This is the real drive loop
                // Condition the X-Y stick inputs
                conditionSticks();
                // Set the motor speeds
                if (drive_mode == 0) {
                    // Tank drive - motor speeds are stick Y for both left and right
                    traction.setTankSpeeds(left_y, right_y, (left_x + right_x)/2.0);
                } else if (drive_mode == 1) {
                    // right stick is forward/back and side-to-side with no rotation,
                    // left X is rotation.
                    traction.setSpeeds(right_y, right_x, left_x);
                } else if (drive_mode == 2) {
                    // left stick is forward/back and side-to-side with no rotation,
                    // right X is rotation.
                    traction.setSpeeds(left_y, left_x, right_x);
                } else if (drive_mode == 3) {
                    // right stick is forward/back and right/left rotation with no
                    // sideways motion, left X is sideways motion.
                    traction.setSpeeds(right_y, left_x,
                            right_x + ((right_x * Math.abs(right_y)) * (traction.getAutoTurnRate() - 1.0)));
                } else {
                    // left stick is forward/back and right/left rotation with no
                    // sideways motion, right X is sideways motion.
                    traction.setSpeeds(left_y, right_x,
                            left_x + ((left_x * Math.abs(left_y)) * (traction.getAutoTurnRate() - 1.0)));
                }
            }
            telemetry.update();
        }
    }
    private void conditionSticks() {
        left_x = gamepad1.left_stick_x;
        left_y = -gamepad1.left_stick_y;
        right_x = gamepad1.right_stick_x;
        right_y = -gamepad1.right_stick_y;
        if (gamepad1.right_bumper) {
            left_x *= bumper_speed;
            left_y *= bumper_speed;
            right_x *= bumper_speed;
            right_y *= bumper_speed;
        }
    }
}
