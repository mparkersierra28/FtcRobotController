package org.firstinspires.ftc.teamcode.tests.autonomous.examples;
// This OpMode demonstrates how to use PedroPathing to move the robot back and forth
// in a straight line using Paths and a Follower.
// The robot will drive forward a set distance, then backward the same distance, repeating forever.

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
@Autonomous (name = "Line Pedro Example", group = "Examples")
public class LineExample extends OpMode {

    // The distance (in inches, cm, or your chosen unit depending on your config)
    // that the robot will travel forward and backward.
    public static double DISTANCE = 40;

    // Keeps track of whether the robot is currently moving forward.
    // Starts as true, meaning the first path will go forward.
    private boolean forward = true;

    // Stores the two paths we’ll use: one going forward and one going backward.
    private Path forwards;
    private Path backwards;

    private TelemetryManager telemetryM;

    public static Follower follower;


    @Override
    public void init() {
        // Runs once when the OpMode is initialized.
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        System.out.println("We just initialized the OpMode!");
    }


    /**
     * init_loop() runs repeatedly while the driver station is on INIT but before START is pressed.
     *
     * This is useful for displaying info, debugging, or preparing subsystems.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);

        // Update the follower so it can do things like odometry updates before the match starts.
        follower.update();

        // Draw the current robot position on the field visualization.
        drawCurrent();
    }


    @Override
    public void start() {
        // Runs once when the driver hits PLAY on the driver station.

        // Turns on all the PIDF controllers used by the follower (translation, heading, etc).
        follower.activateAllPIDFs();

        // Create a forward path:
        // A BezierLine is the simplest path, just a straight line between two Poses.
        // In this case, from (0,0) to (DISTANCE, 0) → a straight line along the X axis.
        forwards = new Path(new BezierLine(new Pose(0, 0), new Pose(DISTANCE, 0)));

        // Tell the robot to keep a constant heading of 0 degrees while following this path.
        forwards.setConstantHeadingInterpolation(0);

        // Create the backward path:
        // This is just the reverse of the forward path, from (DISTANCE, 0) back to (0, 0).
        backwards = new Path(new BezierLine(new Pose(DISTANCE, 0), new Pose(0, 0)));

        // Maker it turn/look the other way on the backwards path.
        // It has to be in radians so convert degrees to radians
        backwards.setConstantHeadingInterpolation(toRadians(180));

        // Start following the forward path first.
        follower.followPath(forwards);
    }


    /**
     * loop() runs continuously after start() until the OpMode ends.
     * This is where the robot actually executes movement and swaps between paths.
     */
    @Override
    public void loop() {
        // Update the follower so it can compute motor powers and drive the robot.
        follower.update();

        // Draw the robot’s current position and path history on the field visualization.
        drawCurrentAndHistory();

        // Check if the robot has finished following its current path.
        if (!follower.isBusy()) {
            if (forward) {
                // If we were going forward, switch to going backward next.
                forward = false;
                follower.followPath(backwards);
            } else {
                // If we were going backward, switch to going forward again.
                forward = true;
                follower.followPath(forwards);
            }
        }

        // Print whether the robot is currently driving forward to telemetry.
        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}
