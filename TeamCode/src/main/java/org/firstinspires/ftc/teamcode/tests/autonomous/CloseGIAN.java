package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.PreloadInShoot;
import org.firstinspires.ftc.teamcode.software.GiveItAName.MagazinePositionController;
import org.firstinspires.ftc.teamcode.software.GiveItAName.Sorter;
import org.firstinspires.ftc.teamcode.software.TransferData;

public class CloseGIAN extends OpMode {
    private RobotHardware robot;
    private MagazinePositionController magazinePos;
    private Sorter sorter;
    private TelemetryManager telemetryM;
    private Follower follower;

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

    private enum State { GO_SHOOT, SHOOT, INTAKE, PREP_INTAKE, PARK, DONE }
    private State state = State.GO_SHOOT;
    private long stateStartTime = 0;
    private int amountOfLaunch = 0;

    // Sorter
    public static int lVel = 1000;
    public static double magPower = 0.5;
    private static double gateOpen = 0.25;
    private static double gateClosed = 0;

    private int numShots = 0;



    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        magazinePos = new MagazinePositionController(robot);
        sorter = new Sorter(robot, magazinePos);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = robot.setPedroConstants();

    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robot.alliance = RobotHardware.Alliance.RED;
            allianceSelected = true;
        } else if (gamepad1.b) {
            robot.alliance = RobotHardware.Alliance.BLUE;
            allianceSelected = true;
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
            case GO_SHOOT:
                if (pathDone()) {
                    nextState(State.SHOOT);
                }
                break;
            case SHOOT:
                warmUpLauncher();
                if (stateStartTime - System.currentTimeMillis() < 500) break;
                if (!shootBall()) stateStartTime = System.currentTimeMillis();
                if (stateStartTime - System.currentTimeMillis() < 1000) {
                    numShots++;
                    stopLauncher();
                    if (numShots == 1) {
                        follower.followPath(intakeBPath);
                        nextState(State.PREP_INTAKE);
                    }
                    else if (numShots == 2) {
                        follower.followPath(goPark);
                        nextState(State.PARK);
                    }

                }
                break;
            case PREP_INTAKE:
                if (pathDone()) {
                    follower.followPath(intakeEPath);
                    nextState(State.INTAKE);
                }
                break;
            case INTAKE:
                if (runIntake() || pathDone()) {
                    follower.followPath(launchSecond3);
                    nextState(State.GO_SHOOT);
                }
                break;
            case PARK:
                if (pathDone()) nextState(State.DONE);
                break;
            case DONE:
                requestOpModeStop();
                break;
        }

    }
    private void flipForAlliance() {
        if (robot.alliance == RobotHardware.Alliance.BLUE) {
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

    private Path makePath(Pose start, Pose end, double startH, double endH, double velocity) {
        Path p = new Path(new BezierLine(start, end));
        p.setLinearHeadingInterpolation(Math.toRadians(startH), Math.toRadians(endH));
        p.setVelocityConstraint(velocity);
        return p;
    }
    private void nextState(State next) {
        state = next;
        stateStartTime = System.currentTimeMillis();
    }
    private boolean pathDone() {
        return !follower.isBusy();
    }

    private boolean warmUpLauncher() {
        robot.launcher.setVelocity(lVel);

        return sorter.moveNextPatternToExit(magPower);
    }
    private boolean shootBall() {
        if (magazinePos.isBusy()) return true;

        robot.gateS.setPosition(gateOpen);

        if (sorter.checkAndClearExit()) {
            robot.gateS.setPosition(gateClosed);
            return warmUpLauncher();
        }
        return true;
    }
    private void stopLauncher() {
        robot.launcher.setPower(0);
    }
    private boolean runIntake() {
        // Todo add intake
        if (magazinePos.isBusy()) return false;
        if (!sorter.detectIntake()) return false;
        boolean isFull = !sorter.moveEmptyToIntake(magPower);
        if (isFull) {
            stopIntake();
        }
        return isFull;
    }
    private void stopIntake() {
        // Todo stop intake
    }
}
