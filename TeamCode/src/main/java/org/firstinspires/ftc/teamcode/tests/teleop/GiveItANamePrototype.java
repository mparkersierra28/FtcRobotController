package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private CameraQR camera;
    private Sorter sorter;
    private MagazinePositionController magazinePos;

    public TelemetryManager telemetryM;

    public static int lVel = 1000;
    public static double magPower = 0.5;

    // Menu
    private boolean fieldCentric;
    private boolean twoPlayers = false;
    private boolean aMenuPressed1;
    private boolean bMenuPressed1;
    private boolean aMenuPressed2;

    // Toggles
    private boolean bPressed = false;
    private boolean aPressed = false;
    private int aToggleState = 1;

    // Intake
    private boolean runningIntake = false;
    private long stopIntakeStartTime = 0;
    private static double intakeLogicDelay = 2000; // how long to keep running after stop

    // Launching
    private boolean warmUpTriggered = false;
    private static double gateOpen = 0.25;
    private static double gateClosed = 0;

    public static int goalXPos = 140, goalYPos = 4;

    public static double intakePow = 0.2;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Get the driving class
        drive = new MecanumDrive(robot);

        camera = new CameraQR(robot);

        physics = new Physics(robot);

        sorter = new Sorter(robot, magazinePos);

        magazinePos = new MagazinePositionController(robot);

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
            robot.alliance = (robot.alliance == RobotHardware.Alliance.RED)
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

        telemetry.addData("Menu: Alliance", robot.alliance == RobotHardware.Alliance.RED ? "RED" : "BLUE");
        telemetry.addLine("Press B to toggle");

        telemetry.addData("Menu: Two Player Mode", twoPlayers ? "YES" : "NO");
        telemetry.addLine("Gamepad2: Press A to toggle");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setFieldCentric(fieldCentric);

        telemetry.addData("Field Centric is", fieldCentric ? "ON" : "OFF");
        telemetry.addData("Alliance is", robot.alliance);
        telemetry.addData("Two Player Mode is ", twoPlayers ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamePad1Controls();

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

        if (gamepad1.b && !bPressed) {
            runningIntake = !runningIntake;
            if (!runningIntake) {
                stopIntake();
                stopIntakeStartTime = System.currentTimeMillis(); // mark when stop starts
            }
        }
        bPressed = gamepad1.b;

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
            case 1:
                if (!warmUpTriggered) {
                    warmUpLauncher();   // runs only once
                    warmUpTriggered = true;
                }
                break;
            case 2:
                shootBall();  // runs continuously while in state 2
                break;
            case 3:
                stopAllLaunching();
                aToggleState = 0;
                break;
        }

    }

    private void intake() {
        robot.intakeLS.setPower(intakePow);
        robot.intakeRS.setPower(intakePow);

        if (magazinePos.isBusy()) return;
        if (!sorter.detectIntake()) return;
        if (!sorter.moveEmptyToIntake(magPower)) {
            runningIntake = false;
            stopIntake();
        }

    }
    private void stopIntake() {
        robot.intakeLS.setPower(0);
        robot.intakeRS.setPower(0);
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
    private void warmUpLauncher() {
        robot.launcher.setVelocity(lVel);

        if (!sorter.moveNextPatternToExit(magPower)) aToggleState = 3;
    }
    private void shootBall() {
        if (magazinePos.isBusy()) return;

        robot.gateS.setPosition(gateOpen);

        if (sorter.checkAndClearExit()) {
            robot.gateS.setPosition(gateClosed);
            warmUpLauncher();
        }

    }
    private void stopAllLaunching() {
        robot.gateS.setPosition(gateClosed);
        robot.launcher.setPower(0);

        warmUpTriggered = false;

    }
}
