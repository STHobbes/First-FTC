package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This class interfaces to Roy and Jeff's Rover Ruckus mecanum drive base. So we know the configuration and will get
 * it from the hardware map at initialization.
 */
public class MecanumTraction extends TractionBase {

    // This is the physical hardware - motors and sensors - that provide the physical implementation of the traction.
    private BNO055IMU imu_0;    // primary IMU
    private BNO055IMU imu_1;    // Backup IMU
    private DcMotor motor_rf;   // right front motor
    private DcMotor motor_rr;   // right rear motor
    private DcMotor motor_lf;   // left front motor
    private DcMotor motor_lr;   // left rear motor
    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended forward-backward motion when sideways motion is requested.
    private static double V_forward_V_side_bias = 0.0;
    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended tuen motion when sideways motion is requested.
    private static double V_turn_V_side_bias = 0.0;

    // The constants that regulate this program - adjust these to your physical
    // implementation of the drive.
    private static double mtr_accel_min = 0.3;
    private static double mtr_decel_min = 0.1;
    private static double mtr_accel_tics = 600.0;
    private static double mtr_decel_tics = 1200.0;
    private static double mtr_accel_degs = 20.0;
    private static double mtr_decel_degs = 30.0;
    private static double tics_per_inch_forward = 84.0;
    private static double tics_per_inch_sideways = 83.0;

    // tracking the heading of the robot
    double heading;
    int heading_revs = 0;
    double heading_raw_last;
    double start_heading;

    // =================================================================================================================
    // Local support functions
    // =================================================================================================================

    /**
     * Setup a motor.
     *
     * @param motor               (DcMotor) The motor to be setup.
     * @param direction           (DcMotor.Direction) The motor direction.
     * @param run_mode            (DcMotor.RunMode) The run mode for the motor.
     * @param zero_power_behavior (DcMotor.ZeroPowerBehavior) The zero-power behaviour.
     */
    private void lclMotorSetup(DcMotor motor, DcMotor.Direction direction,
                               DcMotor.RunMode run_mode, DcMotor.ZeroPowerBehavior zero_power_behavior) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(run_mode);
        motor.setZeroPowerBehavior(zero_power_behavior);
    }

    /**
     * Reset the drive encoders to 0 and set all motors to RUN_USING_ENCODER,
     * which means that when you set a motor power it is really the motor speed
     * and the motor control libraries use the encoders and PID loops to keep
     * the motors coordinated.
     */
    private void reset_drive_encoders() {
        motor_rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Explicitly set the power (really the speed because the motors are running
     * with encoders) for each of the motors and update the robot heading.
     *
     * @param power_rf (double) The power (speed) for the right-front motor.
     * @param power_rr (double) The power (speed) for the right-rear motor.
     * @param power_lf (double) The power (speed) for the left-front motor.
     * @param power_lr (double) The power (speed) for the left-rear motor.
     */
    private void lclSetPower(double power_rf, double power_rr, double power_lf, double power_lr) {
        motor_rf.setPower(power_rf);
        motor_rr.setPower(power_rr);
        motor_lf.setPower(power_lf);
        motor_lr.setPower(power_lr);

        // anytime power is being changed the heading may change. The IMU goes from
        // -180.0 to 180.0. The discontinuity at 180,-180 is a programming headache.
        // if you rotate through that is takes a bunch of special programming logic
        // to figure out where you are. Instead, we will monitor going through that
        // discontinuity and increment a rotation counter so our heading will start
        // at 0 when the IMU is initialized, and be a continuous function from
        // -infinity to +infinity.
        Orientation angles = imu_0.getAngularOrientation();
        double heading_raw = angles.firstAngle;
        if (heading_raw_last < -140.0 && heading_raw > 0.0) {
            heading_revs -= 1;
        } else if (heading_raw_last > 140.0 && heading_raw < 0.0) {
            heading_revs += 1;
        }
        // Our mental model says clockwise rotation (turning right) is a positive
        // rotation for the power, so we will sign correct heading to match.
        heading = -(heading_revs * 360.0 + heading_raw);
        heading_raw_last = heading_raw;
//        telemetry.addData("revs", heading_revs);
//        telemetry.addData("heading", heading);

    }

    // =================================================================================================================
    //
    // =================================================================================================================
    @Override
    public void initialize(HardwareMap hardware_map) {
        // find the primary and secondary IMUs
        imu_0 = hardware_map.get(BNO055IMU.class, "imu_0");
        imu_1 = hardware_map.get(BNO055IMU.class, "imu_1");

        // find the motors
        motor_rf = hardware_map.get(DcMotor.class, "mtr_rf");
        motor_rr = hardware_map.get(DcMotor.class, "mtr_rr");
        motor_lf = hardware_map.get(DcMotor.class, "mtr_lf");
        motor_lr = hardware_map.get(DcMotor.class, "mtr_lr");

        // initialize the motors
        DcMotor.RunMode run_mode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior at_zero_power = DcMotor.ZeroPowerBehavior.BRAKE;
        lclMotorSetup(motor_rf, DcMotor.Direction.FORWARD, run_mode, at_zero_power);
        lclMotorSetup(motor_rr, DcMotor.Direction.FORWARD, run_mode, at_zero_power);
        lclMotorSetup(motor_lf, DcMotor.Direction.REVERSE, run_mode, at_zero_power);
        lclMotorSetup(motor_lr, DcMotor.Direction.REVERSE, run_mode, at_zero_power);
    }

    @Override
    public void postStartInitialize() {
        // initialize the primary and secondary IMUs
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json";
        imu_params.loggingEnabled = false;
        imu_0.initialize(imu_params);
        imu_1.initialize(imu_params);
        //imu_0.startAccelerationIntegration(new Position(), new Velocity(), 100);
        //imu_1.startAccelerationIntegration(new Position(), new Velocity(), 100);
        while (true) {
            if (imu_0.isGyroCalibrated() && imu_1.isGyroCalibrated()) {
                break;
            }
        }
        // initialize the heading tracking
        heading_revs = 0;
        Orientation angles = imu_0.getAngularOrientation();
        heading_raw_last = angles.firstAngle;
        heading = -heading_raw_last;

    }

    @Override
    public boolean supportsSideways() {
        return true;
    }

    @Override
    public void setSpeeds(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        double scale = 1.0;
        double max = Math.abs(forward) +
                Math.abs(sideways * (Math.abs(V_forward_V_side_bias) + Math.abs(V_turn_V_side_bias) + 1.0)) +
                Math.abs(rotate);
        if (max > 1.0) {
            scale = 1.0 / max;
        }
        // Compute the power to each of the motors
        double power_rf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias - 1.0) -
                        rotate);
        double power_rr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias + 1.0) -
                        rotate);
        double power_lf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias + 1.0) +
                        rotate);
        double power_lr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias - 1.0) +
                        rotate);
        // set the powers to each of the motors
        lclSetPower(power_rf, power_rr, power_lf, power_lr);
    }

    @Override
    public void setTankSpeeds(double left, double right, double sideways) {
        // find a scale that makes sure none of the power components is greater than 1 or less than -1
        double scale = 1.0;
        double max_left = Math.abs(left) + Math.abs(sideways);
        double max_right = Math.abs(right) + Math.abs(sideways);
        if (max_left > 1.0) {
            if (max_right > max_left) {
                scale = 1.0 / max_right;
            } else {
                scale = 1.0 / max_left;
            }
        } else if (max_right > 1.0) {
            scale = 1.0 / max_right;
        }
        // set the powers to each of the motors
        lclSetPower(scale * (right - sideways),
                scale * (right + sideways),
                scale * (left + sideways),
                scale * (left - sideways));
    }

    // =================================================================================================================
    //
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
        // We are going to use the cos and sin of the angle as the forward and sideways speed respectively
        double forward_max_speed = Math.abs(Math.cos(Math.toRadians(degrees)) * max_speed);
        double forward_inches = Math.cos(Math.toRadians(degrees)) * inches;
        double forward_direction_mult = (forward_inches > 0.0) ? 1.0 : -1.0;
        double sideways_max_speed = Math.abs(Math.sin(Math.toRadians(degrees)) * max_speed);
        double sideways_inches = Math.sin(Math.toRadians(degrees)) * inches;
        double sideways_direction_mult = (sideways_inches > 0.0) ? 1.0 : -1.0;
        if (forward_max_speed > sideways_max_speed) {
            double forward_target_tics = tics_per_inch_forward * forward_inches * forward_direction_mult;
            while (true) {
                double current_forward_tics = forward_direction_mult * forward_tics();
                if (current_forward_tics >= forward_target_tics) {
                    break;
                }
                double speed_mult = power_accel_decel(current_forward_tics, forward_target_tics,
                                mtr_accel_min, mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
                setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                        sideways_max_speed * speed_mult * sideways_direction_mult,
                        0.0);
            }
        } else {
            double sideways_target_tics = tics_per_inch_sideways * sideways_inches * sideways_direction_mult;
            while (true) {
                double current_sideways_tics = sideways_direction_mult * sideways_tics();
                if (current_sideways_tics >= sideways_target_tics) {
                    break;
                }
                double speed_mult = power_accel_decel(current_sideways_tics, sideways_target_tics,
                        mtr_accel_min, mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
                setSpeeds(forward_max_speed * speed_mult * forward_direction_mult,
                        sideways_max_speed * speed_mult * sideways_direction_mult,
                        0.0);
            }
        }
        setSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public void rotate(double degrees, double max_rotate_speed) {
        double start_heading = heading;
        // Rotate as specified - there is normally some overshoot.
        turn(degrees);
        // Test the heading and correct for error (overshoot)
        turn(degrees - (heading - start_heading));
    }

    /**
     * Turn the specified number of degrees. This is a local function that
     * may have overshoot. Positive is clockwise and negative is counter-clockwise.
     *
     * @param degrees (double) The number of degrees to be rotated.
     */
    private void turn(double degrees) {
        double direction_mult = (degrees > 0.0) ? 1.0 : -1.0;
        double start_heading = heading;
        double target = degrees * direction_mult;
        while (true) {
            double current = direction_mult * (heading - start_heading);
            if (current >= target) {
                break;
            }
            setSpeeds(0.0, 0.0,
                    direction_mult * power_accel_decel(current, target, mtr_accel_min, mtr_decel_min,
                            mtr_accel_degs, mtr_decel_degs));
        }
        setSpeeds(0.0, 0.0, 0.0);
    }
}
