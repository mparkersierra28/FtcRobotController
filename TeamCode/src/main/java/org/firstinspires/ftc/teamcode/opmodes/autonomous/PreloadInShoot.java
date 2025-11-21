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

@Configurable
@Autonomous(name = "Preload & Shoot", group = "1")
public class PreloadInShoot extends OpMode {

    private RobotHardware robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    private RobotHardware.Alliance alliance = RobotHardware.Alliance.BLUE;
    private boolean allianceSelected = false;

    // Tunables: The speed the robot is moving; the speed of the intake's rotation; how fast the robot shoots up close; the power of the servos.
    public static double movVel = 15, intakeVel = 15, closeVelocity = 700, servoPower = 1;
    public static double spinUpTime = 2000, WAIT_BEFORE_LAUNCH_MS = 2000;
    public static int launchingDuration = 4000;
    public static double lErrorMargin = 60;
    public static double gateSPower = 0.2;

    // RED baseline coords
    public static double startX = 0, startY = 0, startHeading = 0;
    public static double shootingX = -29, shootingY = 31, shootingHeading = -50;
    public static double intakeX = -41, intakeBY = 36, intakeEY = -3, intakeHeading = 90;
    public static double finalX = -56, finalY = 31, finalHeading = -90;

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

        telemetry.addLine("Press A for RED, B for BLUE");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            alliance = RobotHardware.Alliance.RED;
            allianceSelected = true;
        } else if (gamepad1.b) {
            alliance = RobotHardware.Alliance.BLUE;
            allianceSelected = true;
        }

        telemetry.addData("Alliance", allianceSelected ? alliance : "None");
        telemetry.update();
    }

    @Override
    public void start() {
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
                runServosAt(-0.5);
                robot.thirdUpS.setPower(-0.01);
                if (timeElapsed(spinUpTime)) nextState(State.FIRE);
                break;

            case FIRE:
                if (Math.abs(closeVelocity-robot.launcherR.getVelocity())<=lErrorMargin&&Math.abs(closeVelocity-robot.launcherL.getVelocity())<=lErrorMargin) {
                    robot.thirdUpS.setPower(gateSPower);
                    runServos();
                } else robot.thirdUpS.setPower(0);
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
                    nextState(State.FOLLOW_PATH);
                }
                break;

            case PARK:
                if (pathDone()) nextState(State.DONE);

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
        if (alliance == RobotHardware.Alliance.BLUE) {
            startY = -startY;
            shootingY = -shootingY;
            intakeBY = -intakeBY;
            intakeEY = -intakeEY;
            finalY = -finalY;
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
        robot.intakeS.setPower(power/10);
    }
}
