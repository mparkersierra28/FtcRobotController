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
@Autonomous(name = "Far Side Preload Unified", group = "Old")
public class FarSidePreloadUnified extends OpMode {

    private RobotHardware robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    // Alliance selection
    private RobotHardware.Alliance alliance = RobotHardware.Alliance.BLUE; // default
    private boolean allianceSelected = false;

    // Motion & launcher parameters
    public static double movVel = 15;
    public static double closeVelocity = 700;
    public static double servoPower = 1;
    public static double spinUpTime = 2000;
    public static long WAIT_BEFORE_LAUNCH_MS = 3000;

    // Base starting coordinates (RED values)
    public static double startX = 0, startY = 31, startHeading = -50;
    public static double path1X = -29, path1Y = 31, path1Heading = -50;
    public static double finalX = -56, finalY = 31, finalHeading = 0;

    private Path launchFirst3;
    private Path goPark;

    // State machine
    private enum State { INIT_PATH, FOLLOW_PATH, ADJUST_POS, RUN_LAUNCHERS, RUN_SERVOS, PARK, DONE }
    private State currentState = State.INIT_PATH;
    private long stateStartTime = 0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = robot.setPedroConstants();

        telemetry.addLine("Press A for RED, B for BLUE");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance selection during init
        if (gamepad1.a) {
            alliance = RobotHardware.Alliance.RED;
            allianceSelected = true;
        } else if (gamepad1.b) {
            alliance = RobotHardware.Alliance.BLUE;
            allianceSelected = true;
        }

        telemetry.addData("Alliance Selected", allianceSelected ? alliance : "None");
        telemetry.update();
    }

    @Override
    public void start() {
        // Flip coordinates & headings for BLUE alliance
        if (alliance == RobotHardware.Alliance.BLUE) {
            startY = -startY;
            path1Y = -path1Y;
            finalY = -finalY;
            startHeading = -startHeading;
            path1Heading = -path1Heading;
            finalHeading = -finalHeading;
        }

        follower.setStartingPose(new Pose(startX, startY, startHeading));

        Pose launchingPos = new Pose(path1X, path1Y);
        launchFirst3 = new Path(new BezierLine(new Pose(startX, startY), launchingPos));
        launchFirst3.setLinearHeadingInterpolation(startHeading, Math.toRadians(path1Heading));
        launchFirst3.setVelocityConstraint(movVel);

        goPark = new Path(new BezierLine(launchingPos, new Pose(finalX, finalY)));
        goPark.setLinearHeadingInterpolation(Math.toRadians(path1Heading), Math.toRadians(finalHeading));
        goPark.setVelocityConstraint(movVel);

        currentState = State.FOLLOW_PATH;
        follower.followPath(launchFirst3);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case FOLLOW_PATH:
                if (follower.isBusy()) follower.update();
                else {
                    follower.update();
                    currentState = State.ADJUST_POS;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case ADJUST_POS:
                telemetryM.debug("Waiting before launchers...");
                follower.update();
                if (System.currentTimeMillis() - stateStartTime >= WAIT_BEFORE_LAUNCH_MS) {
                    currentState = State.RUN_LAUNCHERS;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case RUN_LAUNCHERS:
                telemetryM.debug("Launchers");
                runLaunchers();
                follower.update();
                if (System.currentTimeMillis() - stateStartTime >= spinUpTime) {
                    currentState = State.RUN_SERVOS;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case RUN_SERVOS:
                telemetryM.debug("Servos");
                runServos();
                if (System.currentTimeMillis() - stateStartTime >= 6000) {
                    stopLaunching();
                    follower.followPath(goPark);
                    currentState = State.PARK;
                }
                break;

            case PARK:
                if (follower.isBusy()) follower.update();
                else currentState = State.DONE;
                break;

            case DONE:
                requestOpModeStop();
                break;
        }

        telemetryM.update(telemetry);
    }

    private void runLaunchers() {
        robot.launcherR.setVelocity(closeVelocity);
        robot.launcherL.setVelocity(closeVelocity);
    }

    private void runServos() {
        robot.thirdUpS.setPower(servoPower);
        robot.secondUpS.setPower(servoPower);
        robot.firstUpS.setPower(servoPower);
        robot.intakeS.setPower(servoPower);
    }

    private void stopLaunching() {
        robot.launcherR.setVelocity(0.0);
        robot.launcherL.setVelocity(0.0);
        robot.thirdUpS.setPower(0.0);
        robot.secondUpS.setPower(0.0);
        robot.firstUpS.setPower(0.0);
        robot.intakeS.setPower(0.0);
    }
}
