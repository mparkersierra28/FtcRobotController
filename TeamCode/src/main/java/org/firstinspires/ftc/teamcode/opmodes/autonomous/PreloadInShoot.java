package org.firstinspires.ftc.teamcode.opmodes.autonomous;

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
import org.firstinspires.ftc.teamcode.software.TransferData;

@Configurable
@Autonomous(name = "Preload & Shoot", group = "1")
public class PreloadInShoot extends OpMode {

    private RobotHardware robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    // Tunables: The speed the robot is moving; the speed of the intake's rotation; how fast the robot shoots up close; the power of the servos.
    public static double movVel = 15, intakeVel = 15, closeVelocity = 710, servoPower = 1;
    public static double spinUpTime = 2000, WAIT_BEFORE_LAUNCH_MS = 500;
    public static int launchingDuration = 6000;
    public static double lErrorMargin = 80;
    public static double gateSPower = 0.2;

    // RED baseline coords
//    public static double startX = 0, startY = 0, startHeading = 0;
//    public static double shootingX = -29, shootingY = 31, shootingHeading = -50;
//    public static double intakeX = -41, intakeBY = 36, intakeEY = -3, intakeHeading = 90;
//    public static double finalX = -56, finalY = 31, finalHeading = -90;
    public static double startX = 120, startY = 22, startHeading = 0;
    public static double shootingX = 91, shootingY = 53, shootingHeading = -45;
    public static double intakeX = 79, intakeBY = 58, intakeEY = 19, intakeHeading = 90;
    public static double finalX = 60, finalY = 53, finalHeading = 0;

    private Path launchFirst3, intakeBPath, intakeEPath, launchSecond3, goPark;

    private enum State { FOLLOW_PATH, WAIT_LAUNCH, SPINUP, FIRE, PREPARE_INTAKE, INTAKE, PARK, DONE }
    private State state = State.FOLLOW_PATH;
    private long stateStartTime = 0;
    private int amountOfLaunch = 0;

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
        TransferData.saveAlliance(hardwareMap, (robot.alliance == RobotHardware.Alliance.RED ? 0 : 1));

        flipForAlliance();

        follower.setStartingPose(new Pose(startX, startY, startHeading));

        Pose launchPos = new Pose(shootingX, shootingY);
        Pose intakeBPos = new Pose(intakeX, intakeBY);
        Pose intakeEPos = new Pose(intakeX, intakeEY);
        Pose parkPos = new Pose(finalX, finalY);

        launchFirst3 = makePath(new Pose(startX, startY), launchPos, startHeading, shootingHeading, movVel);
        intakeBPath   = makePath(launchPos, intakeBPos, shootingHeading, intakeHeading, intakeVel);
        intakeEPath   = makePath(intakeBPos, intakeEPos, intakeHeading, intakeHeading, intakeVel);
        launchSecond3 = makePath(intakeEPos, launchPos, intakeHeading, shootingHeading, movVel);
        goPark        = makePath(launchPos, parkPos, shootingHeading, finalHeading, movVel);

        follower.followPath(launchFirst3);
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case FOLLOW_PATH:
                if (pathDone()) {
                    runServosAt(0);
                    nextState(State.WAIT_LAUNCH);
                }
                break;

            case WAIT_LAUNCH:
                if (timeElapsed(WAIT_BEFORE_LAUNCH_MS)) nextState(State.SPINUP);
                break;

            case SPINUP:
                runLaunchers();
                if (amountOfLaunch != 0&& !timeElapsed(1200)) {
                    robot.secondUpS.setPower(-0.5);
                    robot.firstUpS.setPower(-0.1);
                    //robot.thirdUpS.setPower(0.05);

                }

                if (timeElapsed(spinUpTime)) nextState(State.FIRE);
                break;

            case FIRE:
//                if (timeElapsed(800)) {
//                    robot.thirdUpS.setPower(gateSPower);
//                    runServos();
//                }
                robot.thirdUpS.setPower(gateSPower);
                runServos();
                if (timeElapsed(launchingDuration)) {
                    amountOfLaunch++;
                    stopLaunching();
                    if (amountOfLaunch == 2) {
                        follower.followPath(goPark);
                        nextState(State.PARK);
                    } else {
                        follower.followPath(intakeBPath);
                        nextState(State.PREPARE_INTAKE);
                    }

                }
                break;

            case PREPARE_INTAKE:
                if (pathDone()) {
                    runServos();
                    follower.followPath(intakeEPath);
                    nextState(State.INTAKE);
                }
                break;

            case INTAKE:
                if (pathDone()) {
                    follower.followPath(launchSecond3);
                    runLaunchersAt(-0.3);
                    nextState(State.FOLLOW_PATH);
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
        if (robot.alliance == RobotHardware.Alliance.BLUE) {

            startY = 144 - startY;
            shootingY = 144 - shootingY;
            intakeBY = 144 - intakeBY;
            intakeEY = 144 - intakeEY;
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
        //robot.thirdUpS.setPower(servoPower);
        robot.secondUpS.setPower(servoPower);
        robot.firstUpS.setPower(servoPower);
        robot.intakeS.setPower(servoPower);
    }

    private void stopLaunching() {
        runLaunchersAt(0);
        runServosAt(0);
        robot.thirdUpS.setPower(0);
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
}
