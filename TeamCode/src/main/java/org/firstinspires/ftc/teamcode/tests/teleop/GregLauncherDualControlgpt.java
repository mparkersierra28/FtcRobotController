package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;

@Configurable
@TeleOp(name = "Combined Launcher TeleOp Fixed")
public class GregLauncherDualControlgpt extends OpMode {

    private RobotHardware robot;
    private MecanumDrive drive;
    private TelemetryManager telemetryM;

    // --- Gamepad1 Variables ---
    public double launchPower1 = 1.0;
    public double servoPower1 = 1.0;
    private boolean launcherRunning1 = false;
    private boolean feederRunning1 = false;
    private boolean reverseRunning1 = false;
    private boolean prevA1 = false;
    private boolean prevB1 = false;
    private boolean prevX1 = false;
    private long launcherStartTime1 = 0;
    private double speedMultiplier1 = 1.0;
    private boolean servosActive1 = false; // track if servos are running

    // --- Gamepad2 Variables ---
    public double launchPower2 = 1.0;
    public double servoPower2 = 1.0;
    private boolean launchMode2 = false;
    private boolean middleMode2 = false;
    private boolean intakeMode2 = false;
    private boolean allServosMode2 = false;
    private boolean allServosAndLaunchMode2 = false;
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevLB2 = false;
    private double speedMultiplier2 = 1.0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new MecanumDrive(robot);
    }

    @Override
    public void loop() {

        // --- Update speed multipliers ---
        speedMultiplier1 = 1.0 - (0.9 * gamepad1.right_trigger);
        speedMultiplier2 = 1.0 - (0.9 * gamepad2.right_trigger);

        // --- Gamepad1 Robot Drive ---
        drive.update(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
        );

        // --- Gamepad1 left trigger stops everything ---
        if (gamepad1.left_trigger > 0.1) {
            stopAll();
            launcherRunning1 = false;
            feederRunning1 = false;
            reverseRunning1 = false;
            servosActive1 = false;
        }

        // --- Gamepad1 toggles ---
        handleGamepad1Toggles();

        // --- Gamepad1 motor handling ---
        handleGamepad1Motors();

        // --- Gamepad2 left trigger stops everything ---
        if (gamepad2.left_trigger > 0.1) {
            stopAll();
            resetGamepad2Modes();
        }

        // --- Gamepad2 toggles ---
        handleGamepad2ModeToggles();

        // --- Gamepad2 motor handling ---
        handleGamepad2Motors();

        // --- Telemetry ---
        telemetryM.debug("Gamepad1: Launcher=" + launcherRunning1 + " Feeder=" + feederRunning1 + " Reverse=" + reverseRunning1);
        telemetryM.debug("Gamepad2 Modes: Launch=" + launchMode2 + " Middle=" + middleMode2 + " Intake=" + intakeMode2 + " AllServos=" + allServosMode2 + " All+Launch=" + allServosAndLaunchMode2);
        telemetryM.debug("Speed Multipliers: G1=" + String.format("%.2f", speedMultiplier1) + " G2=" + String.format("%.2f", speedMultiplier2));
        telemetryM.update(telemetry);
    }

    // ------------------ Gamepad1 Methods ------------------
    private void handleGamepad1Toggles() {
        // Launcher toggle (A)
        if (gamepad1.a && !prevA1) {
            if (launcherRunning1) {
                launcherRunning1 = false;
                servosActive1 = false;
                stopAll();
            } else {
                launcherRunning1 = true;
                launcherStartTime1 = System.currentTimeMillis();
                feederRunning1 = false;
                reverseRunning1 = false;
                servosActive1 = false;
            }
        }
        prevA1 = gamepad1.a;

        // Feeder toggle (B)
        if (gamepad1.b && !prevB1) {
            if (feederRunning1) {
                feederRunning1 = false;
                servosActive1 = false;
                stopAll();
            } else {
                feederRunning1 = true;
                launcherRunning1 = false;
                reverseRunning1 = false;
                servosActive1 = true;
            }
        }
        prevB1 = gamepad1.b;

        // Reverse toggle (X)
        if (gamepad1.x && !prevX1) {
            if (reverseRunning1) {
                reverseRunning1 = false;
                stopAll();
            } else {
                reverseRunning1 = true;
                launcherRunning1 = false;
                feederRunning1 = false;
                servosActive1 = true;
            }
        }
        prevX1 = gamepad1.x;
    }

    private void handleGamepad1Motors() {
        double adjLaunch = launchPower1 * speedMultiplier1;
        double adjServo = servoPower1 * speedMultiplier1;

        boolean somethingRunning = false;

        if (launcherRunning1) {
            somethingRunning = true;
            robot.launcherR.setPower(adjLaunch);
            robot.launcherL.setPower(adjLaunch);

            // Start servos after 1 second
            if (!servosActive1 && System.currentTimeMillis() - launcherStartTime1 >= 1000) {
                robot.firstUpS.setPower(adjServo);
                robot.secondUpS.setPower(adjServo);
                robot.thirdUpS.setPower(adjServo);
                robot.intakeS.setPower(adjServo);
                servosActive1 = true;
            } else if (servosActive1) {
                // Keep servos running
                robot.firstUpS.setPower(adjServo);
                robot.secondUpS.setPower(adjServo);
                robot.thirdUpS.setPower(adjServo);
                robot.intakeS.setPower(adjServo);
            } else {
                robot.firstUpS.setPower(0);
                robot.secondUpS.setPower(0);
                robot.thirdUpS.setPower(0);
                robot.intakeS.setPower(0);
            }
        } else if (feederRunning1) {
            somethingRunning = true;
            robot.firstUpS.setPower(adjServo);
            robot.secondUpS.setPower(adjServo);
            robot.thirdUpS.setPower(adjServo);
            robot.intakeS.setPower(adjServo);
        } else if (reverseRunning1) {
            somethingRunning = true;
            robot.launcherR.setPower(-adjLaunch / 2);
            robot.launcherL.setPower(-adjLaunch / 2);
            robot.firstUpS.setPower(-adjServo / 4);
            robot.secondUpS.setPower(-adjServo / 4);
            robot.thirdUpS.setPower(-adjServo / 4);
            robot.intakeS.setPower(-adjServo / 8);
        }

        if (!somethingRunning && !(gamepad2.left_trigger > 0.1)) {
            stopAll();
        }
    }

    // ------------------ Gamepad2 Methods ------------------
    private void handleGamepad2ModeToggles() {
        if (gamepad2.a && !prevA2) toggleMode2("launch");
        prevA2 = gamepad2.a;
        if (gamepad2.b && !prevB2) toggleMode2("middle");
        prevB2 = gamepad2.b;
        if (gamepad2.x && !prevX2) toggleMode2("intake");
        prevX2 = gamepad2.x;
        if (gamepad2.y && !prevY2) toggleMode2("allServos");
        prevY2 = gamepad2.y;
        if (gamepad2.left_bumper && !prevLB2) toggleMode2("allAndLaunch");
        prevLB2 = gamepad2.left_bumper;
    }

    private void toggleMode2(String mode) {
        switch (mode) {
            case "launch": launchMode2 = !launchMode2; resetOtherModes2("launch"); break;
            case "middle": middleMode2 = !middleMode2; resetOtherModes2("middle"); break;
            case "intake": intakeMode2 = !intakeMode2; resetOtherModes2("intake"); break;
            case "allServos": allServosMode2 = !allServosMode2; resetOtherModes2("allServos"); break;
            case "allAndLaunch": allServosAndLaunchMode2 = !allServosAndLaunchMode2; resetOtherModes2("allAndLaunch"); break;
        }
    }

    private void resetOtherModes2(String except) {
        if (!except.equals("launch")) launchMode2 = false;
        if (!except.equals("middle")) middleMode2 = false;
        if (!except.equals("intake")) intakeMode2 = false;
        if (!except.equals("allServos")) allServosMode2 = false;
        if (!except.equals("allAndLaunch")) allServosAndLaunchMode2 = false;
    }

    private void handleGamepad2Motors() {
        double direction = gamepad2.right_bumper ? -1.0 : 1.0;
        double adjLaunch = launchPower2 * speedMultiplier2 * direction;
        double adjServo = servoPower2 * speedMultiplier2 * direction;

        boolean somethingRunning = false;

        if (launchMode2) {
            somethingRunning = true;
            robot.launcherR.setPower(adjLaunch);
            robot.launcherL.setPower(adjLaunch);
            robot.thirdUpS.setPower(adjServo);
        } else if (middleMode2) {
            somethingRunning = true;
            robot.firstUpS.setPower(adjServo);
            robot.secondUpS.setPower(adjServo);
        } else if (intakeMode2) {
            somethingRunning = true;
            robot.intakeS.setPower(adjServo);
        } else if (allServosMode2) {
            somethingRunning = true;
            robot.firstUpS.setPower(adjServo);
            robot.secondUpS.setPower(adjServo);
            robot.thirdUpS.setPower(adjServo);
            robot.intakeS.setPower(adjServo);
        } else if (allServosAndLaunchMode2) {
            somethingRunning = true;
            robot.launcherR.setPower(adjLaunch);
            robot.launcherL.setPower(adjLaunch);
            robot.firstUpS.setPower(adjServo);
            robot.secondUpS.setPower(adjServo);
            robot.thirdUpS.setPower(adjServo);
            robot.intakeS.setPower(adjServo);
        }

        if (!somethingRunning) stopAll();
    }

    private void resetGamepad2Modes() {
        launchMode2 = false;
        middleMode2 = false;
        intakeMode2 = false;
        allServosMode2 = false;
        allServosAndLaunchMode2 = false;
    }

    private void stopAll() {
        robot.launcherR.setPower(0);
        robot.launcherL.setPower(0);
        robot.firstUpS.setPower(0);
        robot.secondUpS.setPower(0);
        robot.thirdUpS.setPower(0);
        robot.intakeS.setPower(0);
    }
}
