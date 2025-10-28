package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;

@Configurable
@TeleOp(name = "Greg Launcher", group = "Greg")
public class gregLauncher extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    public static double launchPower = 1.0;
    public static double servoPower = 1.0;
    public boolean FIELD_CENTRIC = false;

    private TelemetryManager telemetryM;
    public double SPEED_MULTIPLIER = 1.0;
    private long launcherStartTime = 0;
    private boolean launcherRunning = false;
    private boolean prevA = false;
    private boolean feederRunning = false;
    private boolean prevB = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new MecanumDrive(robot);
    }

    @Override
    public void init_loop() {
        if (gamepad1.a && !prevA) {
            FIELD_CENTRIC = !FIELD_CENTRIC;
        }
        prevA = gamepad1.a;

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
        double triggerValue = gamepad1.right_trigger;
        SPEED_MULTIPLIER = 1.0 - (0.9 * triggerValue);

        double adjustedLaunchPower = launchPower * SPEED_MULTIPLIER;
        double adjustedServoPower = servoPower * SPEED_MULTIPLIER;

        drive.update(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
        );

        // --- A Toggle (Launcher) ---
        if (gamepad1.a && !prevA) {
            // if feeder was on, turn it off first
            if (feederRunning) {
                feederRunning = false;
                stopAllLaunching();
            }

            launcherRunning = !launcherRunning;
            if (launcherRunning) {
                launcherStartTime = System.currentTimeMillis();
            } else {
                stopAllLaunching();
            }
        }
        prevA = gamepad1.a;

        // --- B Toggle (Feeder) ---
        if (gamepad1.b && !prevB) {
            // if launcher was on, turn it off first
            if (launcherRunning) {
                launcherRunning = false;
                stopAllLaunching();
            }

            feederRunning = !feederRunning;
            if (!feederRunning) {
                stopAllLaunching();
            }
        }
        prevB = gamepad1.b;

        boolean somethingPressed = false;

        // --- Launcher Running ---
        if (launcherRunning) {
            somethingPressed = true;

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
        else if (feederRunning) {
            somethingPressed = true;
            robot.thirdUpS.setPower(adjustedServoPower);
            robot.secondUpS.setPower(adjustedServoPower);
            robot.firstUpS.setPower(adjustedServoPower);
            robot.intakeS.setPower(adjustedServoPower);
        }

        // --- X Override ---
        else if (gamepad1.x) {
            // stop any active running modes first
            if (launcherRunning || feederRunning) {
                launcherRunning = false;
                feederRunning = false;
                stopAllLaunching();
            }

            somethingPressed = true;
            robot.launcherR.setPower(-adjustedLaunchPower / 2);
            robot.launcherL.setPower(-adjustedLaunchPower / 2);
            robot.thirdUpS.setPower(-adjustedServoPower / 4);
            robot.secondUpS.setPower(-adjustedServoPower / 4);
            robot.firstUpS.setPower(-adjustedServoPower / 4);
            robot.intakeS.setPower(-adjustedServoPower / 8);
        }

        // --- Default stop ---
        if (!somethingPressed) {
            stopAllLaunching();
        }

        // --- Telemetry ---
        telemetryM.debug("Press A to toggle launcher ON/OFF (1s delay before servos).");
        telemetryM.debug("Press B to toggle feeder ON/OFF.");
        telemetryM.debug("Press X to reverse motors for human player.");
        telemetryM.update(telemetry);
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
