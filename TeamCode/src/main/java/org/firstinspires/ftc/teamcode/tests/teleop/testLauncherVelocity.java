package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;

@TeleOp(name = "testing launcher speeds")
public class testLauncherVelocity extends OpMode {

    RobotHardware robot;
    MecanumDrive drive;
    private TelemetryManager telemetryM;

    double targetVelocity = 0;          // ticks per second
    double velocityStep = 100;          // how much to change each press
    double maxVelocity = 3000;          // adjust based on motor type
    double minVelocity = 0;

    // --- Edge detection state ---
    boolean upPressedLast = false;
    boolean downPressedLast = false;
    boolean aPressedLast = false;
    boolean bPressedLast = false;

    // --- Toggles ---
    boolean launcherEnabled = false;
    boolean thirdUpEnabled = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new MecanumDrive(robot);

        robot.launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // --- Detect D-pad presses (edge detection) ---
        boolean upPressed = gamepad1.dpad_up;
        boolean downPressed = gamepad1.dpad_down;

        if (upPressed && !upPressedLast) targetVelocity += velocityStep;
        if (downPressed && !downPressedLast) targetVelocity -= velocityStep;

        upPressedLast = upPressed;
        downPressedLast = downPressed;

        // Clamp velocity within bounds
        targetVelocity = Math.max(minVelocity, Math.min(maxVelocity, targetVelocity));

        // --- A button toggle for launcher motors ---
        if (gamepad1.a && !aPressedLast) {
            launcherEnabled = !launcherEnabled;  // toggle on/off
        }

        // --- B button toggle for third motor ---
        if (gamepad1.b && !bPressedLast) {
            thirdUpEnabled = !thirdUpEnabled;  // toggle on/off
        }

        aPressedLast = gamepad1.a;
        bPressedLast = gamepad1.b;

        // --- Apply launcher toggle state ---
        if (launcherEnabled) {
            robot.launcherL.setVelocity(targetVelocity);
            robot.launcherR.setVelocity(targetVelocity);
        } else {
            robot.launcherL.setVelocity(0);
            robot.launcherR.setVelocity(0);
        }

        // --- Apply third motor toggle state ---
        if (thirdUpEnabled) {
            robot.thirdUpS.setPower(1);
        } else {
            robot.thirdUpS.setPower(0);
        }

        // --- Telemetry output ---
        telemetryM.debug("Target Velocity (ticks/s)", targetVelocity);
        telemetryM.debug("Launcher Enabled", launcherEnabled);
        telemetryM.debug("Third Motor Enabled", thirdUpEnabled);
        telemetryM.debug("Left Velocity", robot.launcherL.getVelocity());
        telemetryM.debug("Right Velocity", robot.launcherR.getVelocity());
        telemetryM.debug("Difference", robot.launcherL.getVelocity() - robot.launcherR.getVelocity());
        telemetryM.update(telemetry);
    }
}
