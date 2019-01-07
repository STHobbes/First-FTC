package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The interface for a traction object, which represents the traction (horizontal motion) base
 * of the robot. While different traction configurations have different physical drive
 * configurations, the basic drive operations are:
 * - configure and initialize
 * - get capabilities
 * - set speeds
 * - move at some angle and distance stopping with the robot at the same heading as it was at
 *   the beginning of the move
 * - rotate some degrees with max rotation speed
 */
public interface ITraction {
    /**
     * Initialize the traction implementation, which normally means find the traction motors in the
     * <tt>hardware_map</tt> and initialize them for use. Setup IMUs, etc.
     *
     * @param hardware_map (HardwareMap, readonly) The hardware map for the robot.
     */
    void initialize(HardwareMap hardware_map);

    /**
     * The post-start initialization. This is here for things like IMU calibration which should be delayed until
     * start because of gyro precession, which will change orientation over time and can cause real problems if
     * there is a long delay between robot initialization and the start of play.
     */
    void postStartInitialize();

    /**
     * Does the physical traction mechanism support sideways motion.
     *
     * @return <tt>true</tt> if sideways motion is supported, <tt>false</tt> otherwise.
     */
    boolean supportsSideways();

    /**
     * Set speeds based on the abstraction of robot motion being a combination
     * of forward, sideways, and rotation in the range -1 to 1 where
     * -1 is the fastest the robot can move in the negative direction, 0 is
     * stopped for that direction.
     *
     * @param forward  (double) The forward speed in the range -1 (full speed
     *                 backwards) to 1 (full speed forward).
     * @param sideways (double) The sideways speed in the range -1 (full speed
     *                 left) to 1 (full speed right). NOTE: if the traction does
     *                 not support sideways motion, this argument will be ignored.
     * @param rotate   (double) The rotation speed in the range -1 (full speed
     *                 counter-clockwise) to 1 (full speed clockwise)
     */
    void setSpeeds(double forward, double sideways, double rotate);

    /**
     * Set speed using a tank paradigm of left wheel speed, right wheel speed,
     * and sideways speed.
     *
     * @param left     (double) The left speed in the range -1 (full speed
     *                 backwards) to 1 (full speed forward).
     * @param right    (double) The right speed in the range -1 (full speed
     *                 backwards) to 1 (full speed forward).
     * @param sideways (double) The sideways speed in the range -1 (full speed
     *                 left) to 1 (full speed right). NOTE: if the traction does
     *                 not support sideways motion, this argument will be ignored.
     */
    void setTankSpeeds(double left, double right, double sideways);

    /**
     * Move along the specified heading, the specified distance. If
     * the traction does not support sideways motion then this is implemented as
     * pseudo-move motion using a rotation of <tt>degrees</tt>, a forward
     * move of <tt>inches</tt></tt>, and a rotation of <tt>-degrees</tt> so the robot
     * is facing in the expected forward direction after the move (as though the
     * traction can support any combination of forward-sideways motion without
     * rotation).
     *
     * @param inches    (double) The distance in inches to move along the specified
     *                  direction. This may be a negative distance. NOTE that moving
     *                  in a negative distance is the same as changing the direction
     *                  180 degrees and moving in a positive direction.
     * @param degrees   (double) The direction the robot should move: 0.0 is straight
     *                  ahead; 90.0 is sideways to the right; -90.0 is sideways to the
     *                  left; 180 (-180) is backwards; e.g. 45.0 would be diagonally
     *                  forward and to the right.
     * @param max_speed (double) The maximum speed in the range 0 to 1
     */
    void move(double inches, double degrees, double max_speed);

    /**
     * Rotate the robot by the specified number of degrees. Positive is clockwise
     * and negative is counter-clockwise.
     *
     * @param degrees          (double) The number of degrees to be rotated.
     * @param max_rotate_speed (double) The maximum speed in the range 0 to 1
     */
    void rotate(double degrees, double max_rotate_speed);
}
