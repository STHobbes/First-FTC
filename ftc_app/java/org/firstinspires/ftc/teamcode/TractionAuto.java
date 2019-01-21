package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This is a simple autonomous program in which the robot moves through:
 * <ul>
 *   <li>a square:<ul>
 *     <li>forward 36"</li>
 *     <li>right 36"</li>
 *     <li>backwards 36"</li>
 *     <li>left 36" (back to the starting point)</li>
 *   </ul></li>
 *   <li>an X (inscribed in the previous square):<ul>
 *     <li>forward-right at 45&deg; for 50.91"</li>
 *     <li>backwards 36"</li>
 *     <li>forward-left at 45&deg; for 50.91"</li>
 *     <li>backwards 36"</li>
 *   </ul></li>
 *   <li>spine:<ul>
 *     <li>rotate 360&deg; clockwise</li>
 *     <li>rotate 360&deg; counter-clockwise</li>
 *   </ul></li>
 * </ul>
 */
@Autonomous(name = "TractionAuto", group = "")
public class TractionAuto extends LinearOpMode  {
    // Load the implementation of the program for your physical implementation of the traction here.
//    private ITraction traction = new MecanumTraction();         // Mecanum, no PID heading correction
    private ITraction traction = new MecanumPidTraction();      // Mecanum, with PID heading correction
//    private ITraction traction = new SquareBotTraction();       // SquareBot, no PID heading correction
//    private ITraction traction = new SquareBotPidTraction();    // SquareBot, no PID heading correction

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode()  throws java.lang.InterruptedException {
        // initialize the traction base.
        traction.initialize(this);

        this.waitForStart();
        traction.postStartInitialize();

        // run an autonomous pattern
        // Run in a 3' square
        traction.move(36.0, 0.0, 1.0);
        traction.move(36.0, 90.0, 1.0);
        traction.move(-36.0, 0.0, 1.0);
        traction.move(36.0, -90.0, 1.0);
        Thread.sleep(1000);

        // run in a 3' X within the above square
        traction.move(50.91, 45.0, 1.0);
        traction.move(-36.0, 0.0, 1.0);
        traction.move(50.91, -45.0, 1.0);
        traction.move(-36.0, 0.0, 1.0);
        Thread.sleep(1000);

        // spin (demonstrates crossing the -180 to/from 189 boundary
        traction.rotate(360.0, 1.0);
        traction.rotate(-360.0, 1.0);
        Thread.sleep(1000);
    }
}
