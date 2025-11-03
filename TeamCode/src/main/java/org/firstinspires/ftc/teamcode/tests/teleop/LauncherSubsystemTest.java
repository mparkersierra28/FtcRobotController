package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Launcher Subsystem Test (Gamepad2)", group = "Greg")
public class LauncherSubsystemTest extends OpMode {
    private RobotHardware robot;
    private TelemetryManager telemetryM;

    public static double launchPower = 1.0;
    public static double servoPower = 1.0;
    public static double SPEED_MULTIPLIER = 1.0;

    // Mode states
    private boolean launchMode = false;
    private boolean middleMode = false;
    private boolean intakeMode = false;
    private boolean allServosMode = false;
    private boolean allServosAndLaunchMode = false;

    // Previous states for edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLB = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        double triggerValue = gamepad2.right_trigger;
        SPEED_MULTIPLIER = 1.0 - (0.9 * triggerValue);

        // If holding right bumper, everything is negative
        double direction = gamepad2.right_bumper ? -1.0 : 1.0;

        double adjustedLaunchPower = launchPower * SPEED_MULTIPLIER * direction;
        double adjustedServoPower = servoPower * SPEED_MULTIPLIER * direction;

        // --- A: Launch mode (toggle) ---
        if (gamepad2.a && !prevA) {
            if (launchMode) {
                launchMode = false;
                stopAll();
            } else {
                resetModes();
                launchMode = true;
            }
        }
        prevA = gamepad2.a;

        // --- B: Middle mode (toggle) ---
        if (gamepad2.b && !prevB) {
            if (middleMode) {
                middleMode = false;
                stopAll();
            } else {
                resetModes();
                middleMode = true;
            }
        }
        prevB = gamepad2.b;

        // --- X: Intake mode (toggle) ---
        if (gamepad2.x && !prevX) {
            if (intakeMode) {
                intakeMode = false;
                stopAll();
            } else {
                resetModes();
                intakeMode = true;
            }
        }
        prevX = gamepad2.x;

        // --- Y: All servos (toggle) ---
        if (gamepad2.y && !prevY) {
            if (allServosMode) {
                allServosMode = false;
                stopAll();
            } else {
                resetModes();
                allServosMode = true;
            }
        }
        prevY = gamepad2.y;

        // --- LB: All servos + launch (toggle) ---
        if (gamepad2.left_bumper && !prevLB) {
            if (allServosAndLaunchMode) {
                allServosAndLaunchMode = false;
                stopAll();
            } else {
                resetModes();
                allServosAndLaunchMode = true;
            }
        }
        prevLB = gamepad2.left_bumper;

        boolean somethingRunning = false;

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

        if (!somethingRunning) stopAll();

        // --- Telemetry ---
        telemetryM.debug("Gamepad2 A: Launch (motors + 3rd servo)");
        telemetryM.debug("Gamepad2 B: Middle (1st + 2nd servos)");
        telemetryM.debug("Gamepad2 X: Intake (intake servo)");
        telemetryM.debug("Gamepad2 Y: All Servos");
        telemetryM.debug("Gamepad2 LB: All Servos + Launch");
        telemetryM.debug("Hold RB: Reverse Direction");
        telemetryM.debug("Trigger scales speed (%.2f)", SPEED_MULTIPLIER);
        telemetryM.update(telemetry);
    }

    private void stopAll() {
        robot.launcherR.setPower(0);
        robot.launcherL.setPower(0);
        robot.firstUpS.setPower(0);
        robot.secondUpS.setPower(0);
        robot.thirdUpS.setPower(0);
        robot.intakeS.setPower(0);
    }

    private void resetModes() {
        launchMode = false;
        middleMode = false;
        intakeMode = false;
        allServosMode = false;
        allServosAndLaunchMode = false;
        stopAll();
    }
}
