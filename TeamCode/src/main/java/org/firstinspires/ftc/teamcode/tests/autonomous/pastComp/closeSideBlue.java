package org.firstinspires.ftc.teamcode.tests.autonomous.pastComp;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Disabled
@Configurable
@Autonomous(name = "Close Side Intake Blue", group = "Old")
public class closeSideBlue extends OpMode {
    private RobotHardware robot;
    private static double farVelocity = 900;
    public static double intakeVel = 20;
    private double servoPower = 1;

    public static double startX = 0, startY = 0, startHeading = 0;
    public static double path1X = 22, path1Y = -8, path1Heading = -90;
    public static double path2X = 22, path2Y = 30, path2Heading = -90;
    public static double finalX = 24, finalY = 0, finalHeading = 0;

    private Path getIntoIntakePos;
    private Path intakeBalls;
    private Path goPark;
    private Follower follower;
    private TelemetryManager telemetryM;

    // States for the loop
    private enum State { INIT_PATH, POSITIONING_PATH, INTAKE_PATH, RUN_LAUNCHERS, RUN_SERVOS, PARK, DONE }
    private State currentState = State.INIT_PATH;
    private long stateStartTime = 0;

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

        Pose positioningPos = new Pose(path1X, path1Y);
        Pose intakingPos = new Pose(path2X, path2Y);
        Pose parkingPos = new Pose(finalX, finalY);

        //follower.activateAllPIDFs();
        getIntoIntakePos = new Path(new BezierLine(new Pose(startX, startY), positioningPos));
        getIntoIntakePos.setLinearHeadingInterpolation(startHeading, Math.toRadians(path1Heading));

        intakeBalls = new Path(new BezierLine(positioningPos, intakingPos));
        intakeBalls.setLinearHeadingInterpolation(Math.toRadians(path1Heading), Math.toRadians(path2Heading));
        intakeBalls.setVelocityConstraint(intakeVel);

        goPark = new Path(new BezierLine(intakingPos, parkingPos));
        goPark.setLinearHeadingInterpolation(Math.toRadians(path2Heading), Math.toRadians(finalHeading));


        currentState = State.POSITIONING_PATH; // start path following
        follower.followPath(getIntoIntakePos);
    }

    @Override
    public void loop() {

        switch (currentState) {

            case POSITIONING_PATH:
                if (follower.isBusy()) {
                    follower.update();
                } else {
                    currentState = State.INTAKE_PATH;
                    runServos();
                    follower.followPath(intakeBalls);
                }
                break;

            case INTAKE_PATH:
                telemetryM.debug("Servos");
                if (follower.isBusy()) {
                    follower.update();
                } else {
                    currentState = State.PARK;
                    sleep(1000);
                    stopLaunching();
                    follower.followPath(goPark);
                }
                break;
            case PARK:
                if (follower.isBusy()) {
                    follower.update();
                } else {
                    currentState = State.DONE;
                    follower.followPath(intakeBalls);
                }
                break;
            case DONE:
                // Finished autonomous, do nothing
                requestOpModeStop();
                break;
        }
        telemetryM.update(telemetry);
    }

    private void runLaunchers() {
        robot.launcherR.setVelocity(farVelocity);
        robot.launcherL.setVelocity(farVelocity);
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
