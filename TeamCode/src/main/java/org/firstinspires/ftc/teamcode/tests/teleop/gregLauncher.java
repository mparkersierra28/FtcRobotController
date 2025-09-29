package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Configurable
@TeleOp (name = "Greg Launcher")
public class gregLauncher extends OpMode {
    private RobotHardware robot;

    public double launchPower = 1.0;
    public double servoPower = 1.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private TelemetryManager telemetryM;
    public double DEADZONE = 0.1;
    public double SPEED_MULTIPLIER = 1.0;

    private boolean joystickActive = false;
    private double strafe = 0;
    private double forward = 0;
    private double rotate = 0;
    private double leftBackPower = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;
    private long launcherStartTime = 0;
    private boolean launcherRunning = false;   // toggle state
    private boolean prevA = false;             // track button edge


    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void loop() {

        mecanumMovement();

        // Toggle logic
        if (gamepad1.a && !prevA) {
            if (!launcherRunning) {
                // Turning ON
                launcherRunning = true;
                launcherStartTime = System.currentTimeMillis();
            } else {
                // Turning OFF
                launcherRunning = false;
                robot.launcherR.setPower(0.0);
                robot.launcherL.setPower(0.0);
                robot.lastS.setPower(0.0);
                robot.LastS.setPower(0.0);
            }
        }
        prevA = gamepad1.a;

        // If running, keep motors on and handle servo delay
        if (launcherRunning) {
            robot.launcherR.setPower(launchPower);
            robot.launcherL.setPower(launchPower);

            if (System.currentTimeMillis() - launcherStartTime >= 1000) {
                robot.lastS.setPower(servoPower);
                robot.LastS.setPower(servoPower);
            } else {
                robot.lastS.setPower(0.0);
                robot.LastS.setPower(0.0);
            }
        }

        // Power adjust
        if (gamepad1.dpad_up && !prevDpadUp) {
            launchPower += 0.05;
        } else if (gamepad1.dpad_down && !prevDpadDown) {
            launchPower -= 0.05;
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        // telemetry
        telemetryM.debug("Press A to toggle launcher ON/OFF (1s delay before servos).");
        telemetryM.debug("Press Up for increased power.\nPress Down for decreased power");
        telemetryM.debug("Current Power: " + launchPower);
        telemetryM.update(telemetry);
    }
    private void mecanumMovement() {
        // Joystick values
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;
        forward = -(leftY + rightY);
        forward = Range.clip(forward, -1.0, 1.0);

        // Deadzone check
        joystickActive = Math.abs(strafe) > DEADZONE ||
                Math.abs(forward) > DEADZONE ||
                Math.abs(rotate) > DEADZONE;

        if (joystickActive) {
                robotCentricDrive(strafe, forward, rotate);

        } else {
            leftBackPower = 0;
            leftFrontPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;

            applyMotorPowers();
        }
    }
    private void robotCentricDrive(double strafe, double forward, double rotate) {
        // Standard mecanum drive (no heading correction)
        leftBackPower = (forward - strafe + rotate) * SPEED_MULTIPLIER;
        leftFrontPower = (forward + strafe + rotate) * SPEED_MULTIPLIER;
        rightFrontPower = (forward - strafe - rotate) * SPEED_MULTIPLIER;
        rightBackPower = (forward + strafe - rotate) * SPEED_MULTIPLIER;

        applyMotorPowers();
    }



    private void applyMotorPowers() {
        // Normalize if any power is > 1
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));
        leftFrontPower  /= max;
        rightFrontPower /= max;
        leftBackPower   /= max;
        rightBackPower  /= max;

        // Send to motors
        robot.lb.setPower(leftBackPower);
        robot.lf.setPower(leftFrontPower);
        robot.rf.setPower(rightFrontPower);
        robot.rb.setPower(rightBackPower);
    }
}
