package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is an extension of MecanumTraction that tracks the autonomous operation heading (expected heading) and is
 * always doing expected heading adjustments through a PID loop. NOTE: in this implementations, all of the hooks for
 * the complete PID implementation are in place, however, only the P part is used. We found this to work well
 * enough that we did not used the I and D parts of the PID.
 */
public class MecanumPidTraction extends MecanumTraction {

    private double expected_heading;    // the expected heading accumulated from  move() and rotate() commands
    private double kp = 0.1;    // The proportional multiplier of the PID loop
//    private double ki = 0.1;    // The integral multiplier of the PID loop
//    private double kd = 0.1;    // The derivative multiplier of the PID loop

    // =================================================================================================================
    // Local support functions
    // =================================================================================================================
    // All local support functions are inherited from MecanumTraction

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

    private double sideways_tics() {
        return ((motor_rr.getCurrentPosition() + motor_lf.getCurrentPosition()) -
                (motor_rf.getCurrentPosition() + motor_lr.getCurrentPosition()));
    }

    @Override
    public void move(double inches, double degrees, double max_speed) {
        reset_drive_encoders();
        // We are going to use the cos and sin of the angle as the forward and
        // sideways speed respectively. NOTE: Math methods that would be called
        // multiple times in an expanded form are called once to minimize overhead
        // in the loop.
        //
        // During the move we will use a PID controller to maintain the
        // heading as the expected heading
        double radians = Math.toRadians(degrees);
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);
        double forward_max_speed = Math.abs(cos * max_speed);
        double forward_inches = cos * inches;
        double forward_direction_mult = (forward_inches > 0.0) ? 1.0 : -1.0;
        double sideways_max_speed = Math.abs(sin * max_speed);
        double sideways_inches = sin * inches;
        double sideways_direction_mult = (sideways_inches > 0.0) ? 1.0 : -1.0;

//        long last_step = 0;
//        double last_error = expected_heading - heading;
//        double error_sum = 0.0;
        if (forward_max_speed > sideways_max_speed) {
            double forward_target_tics = tics_per_inch_forward * forward_inches *
                    forward_direction_mult;
            while (true) {
                double current_forward_tics = forward_direction_mult * forward_tics();
                if (current_forward_tics >= forward_target_tics) {
                    break;
                }
//                long now = System.currentTimeMillis();
//                long delta_time = (0 != last_step) ? now - last_step : 0;
                double error = expected_heading - heading;
//                error_sum += error * delta_time;
//                double dErr = (0 == delta_time) ? 0.0 : (error - last_error) / delta_time;
                double rotate = kp * error /** + ki * error_sum + kd * dErr **/;
//                last_error = error;
//                last_step = now;
                double speed_mult = power_accel_decel(current_forward_tics,
                        forward_target_tics, mtr_accel_min,
                        mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
                setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                        sideways_max_speed * speed_mult * sideways_direction_mult,
                        max_speed * speed_mult * rotate);
            }
        } else {
            double sideways_target_tics = tics_per_inch_sideways * sideways_inches * sideways_direction_mult;
            while (true) {
                double current_sideways_tics = sideways_direction_mult * sideways_tics();
                if (current_sideways_tics >= sideways_target_tics) {
                    break;
                }
//                long now = System.currentTimeMillis();
//                long delta_time = (0 != last_step) ? now - last_step : 0;
                double error = expected_heading - heading;
//                error_sum += error * delta_time;
//                double dErr = (0 == delta_time) ? 0.0 : (error - last_error) / delta_time;
                double rotate = kp * error /** + ki * error_sum + kd * dErr **/;
//                last_error = error;
//                last_step = now;
                double speed_mult = power_accel_decel(current_sideways_tics, sideways_target_tics,
                        mtr_accel_min, mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
                setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                        sideways_max_speed * speed_mult * sideways_direction_mult,
                        max_speed * speed_mult * rotate);
            }
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
