## Imports Overview

## Follower
import com.pedropathing.follower.Follower;

Purpose: Core class that ties your robot hardware to the pathing system.

Responsibilities:
Track the robot’s current position (via Pose or encoders).
Follow paths defined with Path, BezierCurve, or BezierLine.
Provide utility functions like update() and telemetryDebug().

Usage:
Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
follower.followPath(myPathChain);
follower.update();  // call in loop

## Pose
import com.pedropathing.localization.Pose;

Purpose: Represents the robot’s position and heading.

Fields:
x, y (position in inches)
heading (radians)

Usage: Can be used to track current robot state or set start position for paths.
Pose start = new Pose(0, 0, 0); // x=0, y=0, heading=0
follower.setPose(start);

## BezierCurve
import com.pedropathing.pathgen.BezierCurve;

Purpose: Defines a smooth curved path between a start, control, and end point.

Usage: Creates curved trajectories that are smoother than straight lines.
BezierCurve curve = new BezierCurve(
new Point(0, 0, Point.CARTESIAN),
new Point(10, 0, Point.CARTESIAN),  // control point
new Point(10, 10, Point.CARTESIAN)  // end point
);

## BezierLine

import com.pedropathing.pathgen.BezierLine;

Purpose: Special type of curve; essentially a straight line using Bezier math.

Usage: Useful for consistent API with BezierCurve when all paths are built with curves.
BezierLine line = new BezierLine(
new Point(0,0, Point.CARTESIAN),
new Point(10,0, Point.CARTESIAN)
);

## Path
import com.pedropathing.pathgen.Path;

Purpose: A single path segment that can be followed by Follower.

Can be a BezierCurve or BezierLine.

## PathChain
import com.pedropathing.pathgen.PathChain;

Purpose: Combines multiple Path objects into a sequence.

Usage:
PathChain myChain = new PathChain();
myChain.addPath(curve1);
myChain.addPath(line1);
follower.followPath(myChain);

## Point
import com.pedropathing.pathgen.Point;

Purpose: Defines a position in a 2D plane (x, y) and optional heading.

Types:
Point.CARTESIAN – standard x/y in inches.
Point.POLAR – (distance, angle) relative to a reference.

Usage:
Point start = new Point(0,0, Point.CARTESIAN);
Point end = new Point(10, 10, Point.CARTESIAN);

## Timer
import com.pedropathing.util.Timer;

Purpose: Utility class for timing events in autonomous.

Usage:
Timer timer = new Timer();
timer.start();
if(timer.seconds() > 2) {
// do something after 2 seconds
}

## Constants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
FConstants: Robot physical constants (track width, wheel radius, etc.).
LConstants: Follower tuning parameters (PID gains, max velocity/acceleration).
Usage: Passed to Follower on initialization.
@Autonomous(name="ExampleAuto", group="PedroPathing")
public class ExampleAuto extends OpMode {
private Follower follower;
private PathChain path;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        path = follower.pathBuilder()
            .addPath(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(24,0, Point.CARTESIAN)))
            .addPath(new BezierCurve(new Point(24,0, Point.CARTESIAN), new Point(36,12, Point.CARTESIAN), new Point(48,0, Point.CARTESIAN)))
            .build();

        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();
        follower.telemetryDebug(telemetry);
    }
}