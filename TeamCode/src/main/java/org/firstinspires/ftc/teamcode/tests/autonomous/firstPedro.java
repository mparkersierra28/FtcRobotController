package org.firstinspires.ftc.teamcode.tests.autonomous;
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

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@Autonomous (name = "First Pedro")
public class firstPedro extends OpMode {

    // The distance (in inches, cm, or your chosen unit depending on your config)
    // that the robot will travel forward and backward.
    public static double DISTANCE = 48;
    private TelemetryManager telemetryM;

    public static Follower follower;

    private Path goingToFirstBall;


    @Override
    public void init() {
        // Runs once when the OpMode is initialized.
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the robot hardware class
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        // Make sure follower has the correct constants
        follower = robot.setPedroConstants();
    }


    /**
     * init_loop() runs repeatedly while the driver station is on INIT but before START is pressed.
     *
     * This is useful for displaying info, debugging, or preparing subsystems.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);

        // Update the follower so it can do things like odometry updates before the match starts.
        follower.update();
    }


    @Override
    public void start() {
        // Runs once when the driver hits PLAY on the driver station.

        // Turns on all the PIDF controllers used by the follower (translation, heading, etc).
        follower.activateAllPIDFs();
        goingToFirstBall = new Path(new BezierLine(new Pose(0, 0), new Pose(48,0)));
        follower.followPath(goingToFirstBall);
    }


    /**
     * loop() runs continuously after start() until the OpMode ends.
     * This is where the robot actually executes movement and swaps between paths.
     */
    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            requestOpModeStop();
        }

    }
}
