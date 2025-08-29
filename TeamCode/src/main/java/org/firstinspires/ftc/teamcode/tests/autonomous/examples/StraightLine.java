package org.firstinspires.ftc.teamcode.tests.autonomous.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="ExampleAuto", group="PedroPathing")
public class StraightLine extends OpMode {
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