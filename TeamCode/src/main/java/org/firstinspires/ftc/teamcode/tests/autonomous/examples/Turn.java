package org.firstinspires.ftc.teamcode.tests.autonomous.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Pedro Pathing Stuff to import
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Example autonomous OpMode: move forward, turn, then move forward more
 */
@Config
@Autonomous(name="ForwardTurnForward", group="Examples")
public class Turn extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private PathChain simplePath;
    private MultipleTelemetry telemetryA;

    // Configurable distances (in inches) and turn (in degrees)
    public static double FIRST_FORWARD = 24;
    public static double TURN_ANGLE = 90;  // degrees
    public static double SECOND_FORWARD = 12;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        // Initialize PedroPathing follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Build path sequence
        simplePath = follower.pathBuilder()
                // Move forward along X axis
                .addPath(new BezierLine(
                        new Point(0, 0),
                        new Point(FIRST_FORWARD, 0)))
                // Turn in place (uses a point with same x/y but adjusted heading)
                // Move forward along new heading
                .addPath(new BezierLine(
                        new Point(FIRST_FORWARD, 0, Point.CARTESIAN),
                        new Point(FIRST_FORWARD + SECOND_FORWARD, 0, Point.CARTESIAN)))
                .build();

        // Start following the path
        follower.followPath(simplePath);

        // Setup telemetry for driver station + dashboard
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Forward -> Turn -> Forward autonomous initialized");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Update follower to progress along path
        follower.update();

        // Restart path if finished
        if (follower.atParametricEnd()) {
            follower.followPath(simplePath);
        }

        // Show debug telemetry
        follower.telemetryDebug(telemetryA);
    }
}
