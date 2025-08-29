package org.firstinspires.ftc.teamcode.tests.autonomous.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Example autonomous OpMode using PedroPathing in a circle-like path.
 * Demonstrates using Bezier curves with Point.CARTESIAN.
 *
 * Bezier curves: smooth curves defined by a start, control, and end point.
 * Point.CARTESIAN: uses (x, y) coordinates in inches on the field.
 */
@Config
@Autonomous(name = "Circle Example", group = "Examples")
public class Circle extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private PathChain circlePath;
    private MultipleTelemetry telemetryA;

    // Configurable radius of the “circle” path
    public static double RADIUS = 12; // inches

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        // Initialize PedroPathing follower (ties pathing to your motors)
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Build the path using Bezier curves
        // Bezier curves allow smooth, curved motion. Each curve has:
        //   - Start point
        //   - Control point (pulls the curve in a direction)
        //   - End point
        // Using Point.CARTESIAN means coordinates are in inches on the field.
        circlePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(0, 0, Point.CARTESIAN),
                        new Point(RADIUS, 0, Point.CARTESIAN),
                        new Point(RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(
                        new Point(RADIUS, RADIUS, Point.CARTESIAN),
                        new Point(RADIUS, 2*RADIUS, Point.CARTESIAN),
                        new Point(0, 2*RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(
                        new Point(0, 2*RADIUS, Point.CARTESIAN),
                        new Point(-RADIUS, 2*RADIUS, Point.CARTESIAN),
                        new Point(-RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(
                        new Point(-RADIUS, RADIUS, Point.CARTESIAN),
                        new Point(-RADIUS, 0, Point.CARTESIAN),
                        new Point(0, 0, Point.CARTESIAN)))
                .build();

        // Tell the follower to start following the path
        follower.followPath(circlePath);

        // Set up telemetry to show on driver station and FTC Dashboard
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Starting circular path autonomous. Make sure there is enough space!");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Update the follower each loop
        follower.update();

        // If we reached the end, restart the path
        if (follower.atParametricEnd()) {
            follower.followPath(circlePath);
        }

        // Send debug telemetry
        follower.telemetryDebug(telemetryA);
    }
}
