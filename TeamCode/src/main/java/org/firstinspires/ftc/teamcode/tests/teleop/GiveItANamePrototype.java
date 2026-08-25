package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.CameraQR;
import org.firstinspires.ftc.teamcode.software.GiveItAName.MagazinePositionController;
import org.firstinspires.ftc.teamcode.software.GiveItAName.Sorter;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;
import org.firstinspires.ftc.teamcode.software.Physics;


@Configurable
@TeleOp (name = "GIAN Prototype", group = "Test")
public class GiveItANamePrototype extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    private Physics physics;
    //private CameraQR camera;
    private Sorter sorter;
    private MagazinePositionController magazinePos;

    public TelemetryManager telemetryM;

    public static int launchSmallVel = 1430;
    public static int launchBigVel = 2000;
    public static int launchReallyClose = 1000;
    public static double magPower = 0.7;

    // Menu
    private boolean fieldCentric;
    private boolean twoPlayers = false;
    private boolean aMenuPressed1;
    private boolean bMenuPressed1;
    private boolean aMenuPressed2;

    // Toggles
    private boolean bPressed = false;
    private boolean aPressed = false;
    private int aToggleState = 0;

    // Intake
    private boolean runningIntake = false;
    private long stopIntakeStartTime = 0;
    private static double intakeLogicDelay = 2000; // how long to keep running after stop

    // Launching
    private boolean warmUpTriggered = false;

    public static int goalXPos = 140, goalYPos = 4;

    public static double intakePow = 0.5;

    public static double shootMagPow = 0.3;

    public static double closeGatePos = 0.867;
    public static double openGatePos = 0.67;

    private boolean processingBalls = false;
    public static int lErrorMargin = 40;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Get the driving class
        drive = new MecanumDrive(robot);

        //camera = new CameraQR(robot);

        physics = new Physics(robot);

        magazinePos = new MagazinePositionController(robot);

        sorter = new Sorter(robot, magazinePos);
        sorter.reset();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized - Panels Connected");
    }
    @Override
    public void init_loop() {
        // Toggle field-centric mode
        if (gamepad1.a && !aMenuPressed1) {
            fieldCentric = !fieldCentric;
            aMenuPressed1 = true;
        } else if (!gamepad1.a) {
            aMenuPressed1 = false;
        }

        // Toggle alliance (red/blue)
        if (gamepad1.b && !bMenuPressed1) {
            robot.alliance = (robot.isRedAlliance())
                    ? RobotHardware.Alliance.BLUE
                    : RobotHardware.Alliance.RED;
            bMenuPressed1 = true;
        } else if (!gamepad1.b) {
            bMenuPressed1 = false;
        }

        // Toggle two player mode
        if (gamepad2.a && !aMenuPressed2) {
            twoPlayers = !twoPlayers;
            aMenuPressed2 = true;
        } else if (!gamepad2.a) {
            aMenuPressed2 = false;
        }

        telemetry.addData("Menu: Field Centric Mode", fieldCentric ? "ON" : "OFF");
        telemetry.addLine("Press A to toggle");

        telemetry.addData("Menu: Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addLine("Press B to toggle");

        telemetry.addData("Menu: Two Player Mode", twoPlayers ? "YES" : "NO");
        telemetry.addLine("Gamepad2: Press A to toggle");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setFieldCentric(fieldCentric);
        robot.gateS.setPosition(openGatePos);
    }

    @Override
    public void loop() {
        gamePad1Controls();
        gamePad2Controls();

        telemetry.addData("Field Centric is", fieldCentric ? "ON" : "OFF");
        telemetry.addData("Alliance is", robot.alliance);
        telemetryM.addData("Intake", sorter.getIntake());
        telemetryM.addData("Waiting", sorter.getWaiting());
        telemetryM.addData("Exit", sorter.getExit());
        telemetryM.debug("RGB ", robot.intakeSensor.red()+" "+robot.intakeSensor.green()+" "+robot.intakeSensor.blue());
        telemetryM.debug("A State (0 is nothing, 1 is warming up launcher, 2 is launching)", aToggleState);
        telemetryM.addData("Mag Pos", robot.indexer.getCurrentPosition());
        telemetryM.addData("Mag Target Pos", robot.indexer.getTargetPosition());
        telemetryM.debug("Next Up:\n",
                sorter.getPatternColor(0), "\n",
                sorter.getPatternColor(1), "\n",
                sorter.getPatternColor(2));
        telemetryM.update(telemetry);
    }
    private void gamePad1Controls() {
        if (gamepad1.y) {
            //drive.turnToAngle(gamepad1.dpad_down ? clipAngleClose : clipAngleFar);
            drive.turnInDirection(physics.getHeadingError(goalXPos, goalYPos));
            telemetryM.debug("Current Angle: ", robot.odo.getHeading(AngleUnit.DEGREES));
        } else {
            drive.update(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.right_stick_y
            );
        }


        // --- Reset Odo Dir ---
        if (gamepad1.right_bumper) {
            robot.odo.setPosition(new Pose2D(DistanceUnit.INCH, 8.5, 72, AngleUnit.RADIANS, 0));
        }

        if (gamepad1.b && !bPressed) {
            runningIntake = !runningIntake;
            if (!runningIntake) {
                robot.gateS.setPosition(closeGatePos);
                stopIntake();
                stopIntakeStartTime = System.currentTimeMillis(); // mark when stop starts
            } else {
                robot.gateS.setPosition(openGatePos);
            }
        }
        bPressed = gamepad1.b;

        // --- Emergency Stop ---
        if (gamepad1.left_bumper) {
            aToggleState = 3;
            stopIntake();
            stopIntakeStartTime = System.currentTimeMillis();
            runningIntake = false;

            return;
        }

        if (runningIntake) {
            intake();
        } else continueIntakeAfterStop();


        // Handle A button toggle
        boolean aCurrentlyPressed = gamepad1.a;
        if (aCurrentlyPressed && !aPressed) {
            aToggleState++;
        }
        aPressed = aCurrentlyPressed;

        // Run actions based on state
        switch (aToggleState) {
            case 0:
                break;
            case 1:
                if (!warmUpTriggered) {
                    int targetVelocity = gamepad1.dpad_down ? launchBigVel : (gamepad1.dpad_up ? launchReallyClose : launchSmallVel);
                    warmUpLauncher(targetVelocity);   // runs only once
                    warmUpTriggered = true;
                    if (Math.abs(targetVelocity-robot.launcher.getVelocity())<=lErrorMargin){
                        gamepad1.rumble(500);
                    }
                }
                break;
            case 2:
                if (!shootBall()) aToggleState = 3;
                break;
            case 3:
                stopAllLaunching();
                magazinePos.reset(magPower);
                robot.gateS.setPosition(openGatePos);
                processingBalls = false;
                aToggleState = 0;
                break;
        }

    }

    private void intake() {
        robot.intakeServo.setPower(intakePow);

        telemetryM.addData("Mag", 1);
        if (magazinePos.isBusy()) {
            robot.gateS.setPosition(closeGatePos);
            return;
        } else robot.gateS.setPosition(openGatePos);
        telemetryM.addData("Stuck cause color", 1);
        if (!sorter.detectIntake()) return;
        telemetryM.addData("Updating spot", 1);
        sorter.moveEmptyToIntake(magPower);
        telemetryM.addData("Moving ball", 1);

    }
    private void stopIntake() {
        robot.intakeServo.setPower(0);
    }
    // This runs for a few seconds after stopIntake
    private void continueIntakeAfterStop() {
        long elapsed = System.currentTimeMillis() - stopIntakeStartTime;
        if (elapsed < intakeLogicDelay) {
            if (magazinePos.isBusy()) return;
            if (!sorter.detectIntake()) return;
            sorter.moveEmptyToIntake(magPower);
        }
    }
    private void warmUpLauncher(int lVel) {
        robot.launcher.setVelocity(lVel);


    }
    private boolean shootBall() {
        if (magazinePos.isBusy()) return true;

        if (!processingBalls) {
            sorter.shootBall(1);
            processingBalls = true;
        } else {
            processingBalls = false;
            return sorter.moveNextPatternToExit(magPower);
        }

        return true;
    }
    private void stopAllLaunching() {
        robot.launcher.setPower(0);

        warmUpTriggered = false;

    }
    private boolean  upPressed = false;
    private boolean  downPressed = false;
    private void gamePad2Controls() {
        if (!upPressed && gamepad2.dpad_up) sorter.advancePattern();
        upPressed = gamepad2.dpad_up;

        if (!downPressed && gamepad2.dpad_down) {
            sorter.advancePattern();
            sorter.advancePattern();
        }
        downPressed = gamepad2.dpad_down;

    }
}
