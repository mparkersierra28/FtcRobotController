package org.firstinspires.ftc.teamcode.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;

@TeleOp (name="Greg Two Remote", group="Greg")
public class GregLauncherTwoRemotes extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;

    public static double launchPower = 1.0;
    public static double servoPower = 1.0;

    private boolean FIELD_CENTRIC;
    private boolean aMenuPressed;
    private long launcherStartTime = 0;

    // Mode states gamepad1
    private boolean feederRunning1 = false;
    private boolean launcherRunning1 = false;
    private boolean humanRunning1 = false;
    // Buttons 1
    private boolean prevA1 = false;
    private boolean prevB1 = false;
    private boolean prevX1 = false;

    // Buttons 2
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevLB2 = false;

    // Mode states gamepad2
    private boolean launchMode = false;
    private boolean middleMode = false;
    private boolean intakeMode = false;
    private boolean allServosMode = false;
    private boolean allServosAndLaunchMode = false;

    boolean somethingRunning = false;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Get the driving class
        drive = new MecanumDrive(robot);

        telemetry.addData("Status", "Initialized - Panels Connected");
    }
    @Override
    public void init_loop() {
        // Toggle field-centric mode with gamepad1.a
        if (gamepad1.a && !aMenuPressed) {
            FIELD_CENTRIC = !FIELD_CENTRIC;
            aMenuPressed = true; // lock until released
        } else if (!gamepad1.a) {
            aMenuPressed = false; // allow future toggles
        }

        telemetry.addData("Menu: Field Centric Mode", FIELD_CENTRIC ? "ON" : "OFF");
        telemetry.addLine("Press A to toggle");
        telemetry.update();
    }
    @Override
    public void start() {
        drive.setFieldCentric(FIELD_CENTRIC);
    }
    @Override
    public void loop() {
        // Handle driving
        drive.update(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
        );
        somethingRunning = false;
        gamePad1Controls();
        gamepad2Controls();


    }
    private void gamePad1Controls() {
        double triggerValue = gamepad1.right_trigger;
        double speed = 1.0 - (0.9 * triggerValue);

        double adjustedLaunchPower = launchPower * speed;
        double adjustedServoPower = servoPower * speed;

        drive.update(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
        );

        // --- A Toggle (Launcher) ---
        if (gamepad1.a && !prevA1) {
            // if other was on, turn it off first
            if (feederRunning1 || humanRunning1) {
                feederRunning1 = false;
                humanRunning1 = false;
                stopAllLaunching();
            }

            launcherRunning1 = !launcherRunning1;
            if (launcherRunning1) {
                launcherStartTime = System.currentTimeMillis();
            } else {
                stopAllLaunching();
            }
        }
        prevA1 = gamepad1.a;

        // --- B Toggle (Feeder) ---
        if (gamepad1.b && !prevB1) {
            // if launcher was on, turn it off first
            if (launcherRunning1 || humanRunning1) {
                launcherRunning1 = false;
                humanRunning1 = false;
                stopAllLaunching();
            }

            feederRunning1 = !feederRunning1;
            if (!feederRunning1) {
                stopAllLaunching();
            }
        }
        prevB1 = gamepad1.b;

        // --- X Toggle (Human) ---
        if (gamepad1.x && !prevX1) {
            // if launcher was on, turn it off first
            if (launcherRunning1 || feederRunning1) {
                launcherRunning1 = false;
                feederRunning1 = false;
                stopAllLaunching();
            }

            humanRunning1 = !humanRunning1;
            if (!humanRunning1) {
                stopAllLaunching();
            }
        }
        prevX1 = gamepad1.x;

        // --- Launcher Running ---
        if (launcherRunning1) {
            somethingRunning = true;

            robot.launcherR.setPower(adjustedLaunchPower);
            robot.launcherL.setPower(adjustedLaunchPower);

            if (System.currentTimeMillis() - launcherStartTime >= 1000) {
                robot.thirdUpS.setPower(adjustedServoPower);
                robot.secondUpS.setPower(adjustedServoPower);
                robot.firstUpS.setPower(adjustedServoPower);
                robot.intakeS.setPower(adjustedServoPower);
            } else {
                robot.firstUpS.setPower(0.0);
                robot.intakeS.setPower(0.0);
            }
        }

        // --- Feeder Running ---
        else if (feederRunning1) {
            somethingRunning = true;
            robot.thirdUpS.setPower(adjustedServoPower);
            robot.secondUpS.setPower(adjustedServoPower);
            robot.firstUpS.setPower(adjustedServoPower);
            robot.intakeS.setPower(adjustedServoPower);
        }

        // --- Human Running ---
        else if (humanRunning1) {
            somethingRunning = true;
            robot.launcherR.setPower(-adjustedLaunchPower / 2);
            robot.launcherL.setPower(-adjustedLaunchPower / 2);
            robot.thirdUpS.setPower(-adjustedServoPower / 4);
            robot.secondUpS.setPower(-adjustedServoPower / 4);
            robot.firstUpS.setPower(-adjustedServoPower / 4);
            robot.intakeS.setPower(-adjustedServoPower / 8);
        }

    }

    private void gamepad2Controls() {
        double triggerValue = gamepad2.right_trigger;
        double speed = 1.0 - (0.9 * triggerValue);

        // If holding right bumper, everything is negative
        double direction = gamepad2.right_bumper ? -1.0 : 1.0;

        double adjustedLaunchPower = launchPower * speed * direction;
        double adjustedServoPower = servoPower * speed * direction;

        // --- A: Launch mode (toggle) ---
        if (gamepad2.a && !prevA2) {
            if (launchMode) {
                launchMode = false;
                stopAllLaunching();
            } else {
                resetModes();
                launchMode = true;
            }
        }
        prevA2 = gamepad2.a;

        // --- B: Middle mode (toggle) ---
        if (gamepad2.b && !prevB2) {
            if (middleMode) {
                middleMode = false;
                stopAllLaunching();
            } else {
                resetModes();
                middleMode = true;
            }
        }
        prevB2 = gamepad2.b;

        // --- X: Intake mode (toggle) ---
        if (gamepad2.x && !prevX2) {
            if (intakeMode) {
                intakeMode = false;
                stopAllLaunching();
            } else {
                resetModes();
                intakeMode = true;
            }
        }
        prevX2 = gamepad2.x;

        // --- Y: All servos (toggle) ---
        if (gamepad2.y && !prevY2) {
            if (allServosMode) {
                allServosMode = false;
                stopAllLaunching();
            } else {
                resetModes();
                allServosMode = true;
            }
        }
        prevY2 = gamepad2.y;

        // --- LB: All servos + launch (toggle) ---
        if (gamepad2.left_bumper && !prevLB2) {
            if (allServosAndLaunchMode) {
                allServosAndLaunchMode = false;
                stopAllLaunching();
            } else {
                resetModes();
                allServosAndLaunchMode = true;
            }
        }
        prevLB2 = gamepad2.left_bumper;

        // --- Mode actions ---
        if (launchMode) {
            somethingRunning = true;
            robot.launcherR.setPower(adjustedLaunchPower);
            robot.launcherL.setPower(adjustedLaunchPower);
            robot.thirdUpS.setPower(adjustedServoPower);
        } else if (middleMode) {
            somethingRunning = true;
            robot.secondUpS.setPower(adjustedServoPower);
            robot.firstUpS.setPower(adjustedServoPower);
        } else if (intakeMode) {
            somethingRunning = true;
            robot.intakeS.setPower(adjustedServoPower);
        } else if (allServosMode) {
            somethingRunning = true;
            robot.firstUpS.setPower(adjustedServoPower);
            robot.secondUpS.setPower(adjustedServoPower);
            robot.thirdUpS.setPower(adjustedServoPower);
            robot.intakeS.setPower(adjustedServoPower);
        } else if (allServosAndLaunchMode) {
            somethingRunning = true;
            robot.launcherR.setPower(adjustedLaunchPower);
            robot.launcherL.setPower(adjustedLaunchPower);
            robot.firstUpS.setPower(adjustedServoPower);
            robot.secondUpS.setPower(adjustedServoPower);
            robot.thirdUpS.setPower(adjustedServoPower);
            robot.intakeS.setPower(adjustedServoPower);
        }

        if (!somethingRunning) stopAllLaunching();
    }
    private void resetModes() {
        launchMode = false;
        middleMode = false;
        intakeMode = false;
        allServosMode = false;
        allServosAndLaunchMode = false;
        stopAllLaunching();
    }
    private void stopAllLaunching() {
        robot.launcherR.setPower(0.0);
        robot.launcherL.setPower(0.0);
        robot.thirdUpS.setPower(0.0);
        robot.secondUpS.setPower(0.0);
        robot.firstUpS.setPower(0.0);
        robot.intakeS.setPower(0.0);
    }
}
