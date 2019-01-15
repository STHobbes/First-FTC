package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This base class implements a common traction functions that do not support sideways motion. This base
 * transforms sideways and move to turn, move, turn sequences.
 */
public abstract class TractionBase implements ITraction {

    /**
     * Setup a motor.
     *
     * @param motor               (DcMotor) The motor to be setup.
     * @param direction           (DcMotor.Direction) The motor direction.
     * @param run_mode            (DcMotor.RunMode) The run mode for the motor.
     * @param zero_power_behavior (DcMotor.ZeroPowerBehavior) The zero-power behaviour.
     */
    protected void lclMotorSetup(DcMotor motor, DcMotor.Direction direction,
                               DcMotor.RunMode run_mode, DcMotor.ZeroPowerBehavior zero_power_behavior) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(run_mode);
        motor.setZeroPowerBehavior(zero_power_behavior);
    }

    /**
     * This is a ramp-in and ramp-out generator function that returns a power
     * for the current position in the move. This method assumes the start
     * position is 0, end is the target, and current is the current position.
     *
     * @param current (double) The current position in the range 0 to target.
     * @param target (double) The target final position.
     * @param mtr_accel_min (double) The minimum acceleration motor speed - to
     *  assure the robot gets to the target.
     * @param mtr_decel_min (double) The minimum deceleration motor speed - to
     *  assure the robot gets to the target.
     * @param accel (double) The acceleration distance with power at current=0
     *  starting at mtr_min and power at current=accel reaching 1.0.
     * @param decel (double) The deceleration distance with power at
     *  current=target-decel being 1.0 and power at current=target being mtr_min.
     * @return (double) Returns the power that will be in the range
     *  mtr_min to 1.0
     */
    protected double power_accel_decel(double current, double target,
                                     double mtr_accel_min, double mtr_decel_min,
                                     double accel, double decel) {
        if (current <= 0.0) {
            // could happen if there was some robot motion (was hit or coasting)
            // that confused the sensor/logic.
            return mtr_accel_min;
        } else if (current >= target) {
            // could happen if there was some robot motion (was hit or coasting)
            // that confused the sensor/logic.
            return 0.0;
        }
        double mtr_tmp = 1.0;
        if (current < accel) {
            // in the acceleration zone
            mtr_tmp = mtr_accel_min + (1.0 - mtr_accel_min) * (current / accel);
        }
        if (current > target - decel) {
            // in the deceleration zone
            double mtr_tmp_2 = mtr_decel_min +
                    (1.0 - mtr_decel_min) * ((target - current) / decel);
            if (mtr_tmp_2 < mtr_tmp) {
                // The deceleration is less than the acceleration or the 1.0 max.
                mtr_tmp = mtr_tmp_2;
            }
        }
        return mtr_tmp;
    }

    @Override
    public boolean supportsSideways() {
        return false;
    }

    @Override
    public void resetExpectedHeading() { }

   @Override
    public void move(double inches, double degrees, double max_speed) {
        if (0.0 != degrees) { rotate(degrees, max_speed); }
        forward(inches, max_speed);
        if (0.0 != degrees) { rotate(-degrees, max_speed); }

    }

    /**
     * Move forward the specified distance in inches, a negative value is
     * interpreted as a backwards motion. This is and rotate() are the minimal
     * required implementation
     *
     * @param inches    (double) The distance in inches to move forward or backward.
     * @param max_speed (double) The maximum speed in the range 0 to 1
     */
    protected void forward(double inches, double max_speed)
    {
        throw new UnsupportedOperationException("forward() is not supported in this implementation.");
    }
}
