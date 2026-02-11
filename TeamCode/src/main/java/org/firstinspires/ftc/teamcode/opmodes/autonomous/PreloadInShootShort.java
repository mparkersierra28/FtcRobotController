package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.TransferData;

@Configurable
@Autonomous(name = "Preload & Shoot Small", group = "1")
public class PreloadInShootShort extends OpMode {

    private RobotHardware robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    // Tunables: The speed the robot is moving; the speed of the intake's rotation; how fast the robot shoots up close; the power of the servos.
    public static double movVel = 10, intakeVel = 15, closeVelocity = 670, servoPower = 1;
    public static double WAIT_BEFORE_LAUNCH_MS = 500;
    public static int launchingDuration = 5000;
    public static double lErrorMargin = 80;
    public static double gateSPower = 0.2;

    // RED baseline coords
//    public static double startX = 0, startY = 0, startHeading = 0;
//    public static double shootingX = -29, shootingY = 31, shootingHeading = -50;
//    public static double intakeX = -41, intakeBY = 36, intakeEY = -3, intakeHeading = 90;
//    public static double finalX = -56, finalY = 31, finalHeading = -90;
    private double startX = 120, startY = 22, startHeading = 0;
    private double shootingX = 127, shootingY = 50, shootingHeading = -64;
    private double intakeX1 = 79, intakeBY1 = 58, intakeEY1 = 19, intakeHeading = 90;
    private double intakeX2 = 55, intakeBY2 = 58, intakeEY2 = 10;
    private double finalX = 100, finalY = 20, finalHeading = 0;

    private Path launchFirst3, intakeBPath1, intakeEPath1, launchSecond3, intakeBPath2, intakeEPath2, launchThird3, goPark;

    private enum State { FOLLOW_PATH, WAIT_LAUNCH, SPINUP, FIRE, PREPARE_INTAKE, INTAKE, PARK, DONE }
    private State state = State.FOLLOW_PATH;
    private long stateStartTime = 0;
    private int amountOfLaunch = 0;

    private double gatePushStartTime;

    private boolean isAtGatePos = true;
    public static double sGatePos = 0.7;
    public static double sPushPos = 0;
    public static double gateDelay = 200;
    public static double pushDelay = 1500;

    private int ballsShoot = 0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = robot.setPedroConstants();

    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robot.alliance = RobotHardware.Alliance.RED;
        } else if (gamepad1.b) {
            robot.alliance = RobotHardware.Alliance.BLUE;
        }

        telemetry.addLine("Press A for RED, B for BLUE");
        telemetry.addData("Alliance", robot.alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        TransferData.saveAlliance(hardwareMap, (robot.isRedAlliance() ? 0 : 1));

        flipForAlliance();

        follower.setStartingPose(new Pose(startX, startY, startHeading));

        Pose launchPos = new Pose(shootingX, shootingY);
        Pose intakeBPos1 = new Pose(intakeX1, intakeBY1);
        Pose intakeEPos1 = new Pose(intakeX1, intakeEY1);
        Pose intakeBPos2 = new Pose(intakeX2, intakeBY2);
        Pose intakeEPos2 = new Pose(intakeX2, intakeEY2);
        Pose parkPos = new Pose(finalX, finalY);

        launchFirst3 = makePath(new Pose(startX, startY), launchPos, startHeading, shootingHeading, movVel);

        intakeBPath1 = makePath(launchPos, intakeBPos1, shootingHeading, intakeHeading, intakeVel);
        intakeEPath1 = makePath(intakeBPos1, intakeEPos1, intakeHeading, intakeHeading, intakeVel);
        launchSecond3 = makePath(intakeEPos1, launchPos, intakeHeading, shootingHeading, movVel);

        intakeBPath2 = makePath(launchPos, intakeBPos2, shootingHeading, intakeHeading, intakeVel);
        intakeEPath2 = makePath(intakeBPos2, intakeEPos2, intakeHeading, intakeHeading, intakeVel);
        launchThird3 = new Path(new BezierCurve(intakeEPos2, intakeBPos2, launchPos));
        launchThird3.setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(shootingHeading));
        launchThird3.setVelocityConstraint(movVel);
        //launchThird3 = makePath(intakeEPos2, launchPos, intakeHeading, shootingHeading, movVel);

        goPark        = new Path(new BezierCurve(launchPos, new Pose(115, 25), parkPos));
        goPark.setLinearHeadingInterpolation(Math.toRadians(shootingHeading), 0);
        goPark.setVelocityConstraint(movVel);

        follower.followPath(launchFirst3);
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case FOLLOW_PATH:
                runLaunchers();
                if (pathDone()) {
                    runServosAt(0);
                    nextState(State.WAIT_LAUNCH);
                }
                break;

            case WAIT_LAUNCH:
                if (timeElapsed(WAIT_BEFORE_LAUNCH_MS)) nextState(State.FIRE);
                break;

            case FIRE:
                launchBalls();
                if (ballsShoot == 3) {
                    amountOfLaunch++;
                    ballsShoot = 0;
                    stopLaunching();
                    follower.followPath(goPark);
                    nextState(State.PARK);

                }
                break;

            case PARK:
                if (pathDone() && timeElapsed(4000)) nextState(State.DONE);

                break;

            case DONE:
                requestOpModeStop();
                break;
        }
        telemetryM.debug(follower.getPose());
        telemetryM.update(telemetry);
    }

    // === Utility methods ===

    private Path makePath(Pose start, Pose end, double startH, double endH, double velocity) {
        Path p = new Path(new BezierLine(start, end));
        p.setLinearHeadingInterpolation(Math.toRadians(startH), Math.toRadians(endH));
        p.setVelocityConstraint(velocity);
        return p;
    }

    private void flipForAlliance() {
        if (!robot.isRedAlliance()) {

            startY = 144 - startY;
            shootingY = 144 - shootingY;
            intakeBY1 = 144 - intakeBY1;
            intakeEY1 = 144 - intakeEY1;
            intakeBY2 = 144 - intakeBY2;
            intakeEY2 = 144 - intakeEY2;
            finalY = 144 - finalY;

            // For headings, if flipping vertically, invert heading
            startHeading = -startHeading;
            shootingHeading = -shootingHeading;
            intakeHeading = -intakeHeading;
            finalHeading = -finalHeading;
        }
    }


    private boolean pathDone() {
        return !follower.isBusy();
    }

    private boolean timeElapsed(double ms) {
        return System.currentTimeMillis() - stateStartTime >= ms;
    }

    private void nextState(State next) {
        state = next;
        stateStartTime = System.currentTimeMillis();
    }

    private void runLaunchers() {
        robot.launcherR.setVelocity(closeVelocity);
        robot.launcherL.setVelocity(closeVelocity);
    }

    private void runServos() {
        robot.secondUpS.setPower(servoPower);
        robot.firstUpS.setPower(servoPower);
        robot.intakeS.setPower(servoPower);
    }

    private void stopLaunching() {
        runLaunchersAt(0);
        runServosAt(0);
    }

    private void runLaunchersAt(double velocity) {
        robot.launcherR.setVelocity(velocity);
        robot.launcherL.setVelocity(velocity);
    }

    private void runServosAt(double power) {
        //robot.thirdUpS.setPower(power);
        robot.secondUpS.setPower(power);
        robot.firstUpS.setPower(power);
        robot.intakeS.setPower(power);
    }
    private void launchBalls() {
        if (isAtGatePos && System.currentTimeMillis() - pushDelay > gatePushStartTime) {
            robot.gatePush.setPosition(sPushPos);
            robot.secondUpS.setPower(0);
            robot.firstUpS.setPower(servoPower/2);
            isAtGatePos = false;
            gatePushStartTime = System.currentTimeMillis();
        } else if (!isAtGatePos && System.currentTimeMillis() - gateDelay > gatePushStartTime) {
            robot.gatePush.setPosition(sGatePos);
            isAtGatePos = true;
            runServos();
            gatePushStartTime = System.currentTimeMillis();
            ballsShoot++;
        }

    }
}
