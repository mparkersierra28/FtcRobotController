package org.firstinspires.ftc.teamcode.tests.autonomous.pastComp;

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
@Autonomous(name = "Close Side Park Red", group = "Old")
public class closeSidePark extends OpMode {
    private RobotHardware robot;

    public static double startX = 0, startY = 0, startHeading = 0;
    public static double finalX = 22, finalY = 0, finalHeading = 0;
    private Path goPark;
    private Follower follower;
    private TelemetryManager telemetryM;

    // States for the loop

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = robot.setPedroConstants();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(startX, startY, startHeading));
        Pose parkingPos = new Pose(finalX, finalY);

        //follower.activateAllPIDFs();
        goPark = new Path(new BezierLine(new Pose(startX, startY), parkingPos));
        goPark.setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(finalHeading));

        follower.followPath(goPark);
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            requestOpModeStop();
        }
        telemetryM.update(telemetry);
    }
}
