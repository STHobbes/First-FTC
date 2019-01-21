package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SquareBotPidTraction extends SquareBotTraction {

    private double expected_heading;    // the expected heading accumulated from  move() and rotate() commands
    private double kp = 0.05;

    // =================================================================================================================
    // Local support functions
    // =================================================================================================================
    // All local support functions are inherited from SquareBotTraction

    // =================================================================================================================
    // ITraction implementation
    // =================================================================================================================
    @Override
    public void postStartInitialize() {
        super.postStartInitialize();
        expected_heading = heading;
    }
    @Override
    public void resetExpectedHeading() { expected_heading = heading; }

    // =================================================================================================================
    // The real movement of the robot (ITraction metods and support methods)
    // =================================================================================================================
    private double forward_tics() {
        return (motor_rf.getCurrentPosition() + motor_lf.getCurrentPosition() +
                motor_rr.getCurrentPosition() + motor_lr.getCurrentPosition());
    }

    protected void forward(double inches, double max_speed)
    {
        // During the move we will use a PID controller (well, the P part anyway) to maintain the
        // heading as the expected heading
        reset_drive_encoders();
        double direction_mult = (inches > 0.0) ? 1.0 : -1.0;
        double target_tics = tics_per_inch_forward * inches * direction_mult;
        while (true) {
            double current_tics = direction_mult * forward_tics();
            if (current_tics >= target_tics) {
                break;
            }
            double error = expected_heading - heading;
            double rotate = kp * error;
            double speed_mult = power_accel_decel(current_tics, target_tics,
                    mtr_accel_min, mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
            setSpeeds(direction_mult * max_speed * speed_mult,
                    0.0, max_speed * speed_mult * rotate);
        }
        setSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public void rotate(double degrees, double max_rotate_speed) {
        // update the expected heading to reflect the rotation and then rotate to the expected heading
        expected_heading += degrees;
        super.rotate(expected_heading - heading, max_rotate_speed);
    }
}
