package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@TeleOp(name = "Working Chasis with Panels", group = "Teleop")
public class chasisControlWithDashSep06 extends OpMode {
    private RobotHardware robot;

    // Panels instance + telemetry
    private TelemetryManager panelsTelemetry;

    // Configurable variables (show up in Panels UI and are live-tunable)
    public static boolean detailedPanels = false;

    public static double SPEED_MULTIPLIER = 1.0;    // 0.1 to 1.0

    public static double DEADZONE = 0.1;
    public static boolean FIELD_CENTRIC = false;

    // Variables for telemetry
    private boolean joystickActive = false;
    private double strafe = 0;
    private double forward = 0;
    private double rotate = 0;
    private double leftBackPower = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    // Helps create a toggle for A
    boolean aPressed;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Initialize Panels + telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized - Panels Connected");
    }
    @Override
    public void init_loop() {
        // Toggle field-centric mode with gamepad1.a
        if (gamepad1.a && !aPressed) {
            FIELD_CENTRIC = !FIELD_CENTRIC;
            aPressed = true; // lock until released
        } else if (!gamepad1.a) {
            aPressed = false; // allow future toggles
        }

        telemetry.addData("Menu: Field Centric Mode", FIELD_CENTRIC ? "ON" : "OFF");
        telemetry.addLine("Press A to toggle");
        telemetry.update();
    }
    @Override
    public void loop() {
        // Handle driving
        mecanumMovement();

        // Send telemetry to Panels and DS
        updateTelemetry();
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
            if (FIELD_CENTRIC) {
                fieldCentricDrive(strafe, forward, rotate);
            } else {
                robotCentricDrive(strafe, forward, rotate);
            }
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

    private void fieldCentricDrive(double strafe, double forward, double rotate) {
        // --- Update odometry and get heading in radians ---
        robot.odo.update();
        Pose2D robotPosition = robot.odo.getPosition();
        double headingRad = robotPosition.getHeading(AngleUnit.RADIANS);

        // --- Rotate joystick vector by -heading (field -> robot coords) ---
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double x = strafe * cosA - forward * sinA;  // robot-centric strafe
        double y = strafe * sinA + forward * cosA;  // robot-centric forward

        // --- Standard mecanum wheel power equations ---
        leftFrontPower  = (y + x + rotate) * SPEED_MULTIPLIER;
        rightFrontPower = (y - x - rotate) * SPEED_MULTIPLIER;
        leftBackPower   = (y - x + rotate) * SPEED_MULTIPLIER;
        rightBackPower  = (y + x - rotate) * SPEED_MULTIPLIER;

        // Apply to motors
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

    private void updateTelemetry() {
        // Panels telemetry
        if (detailedPanels) {
            panelsTelemetry.debug("Joystick Active", joystickActive);
            panelsTelemetry.debug("Strafe", strafe);
            panelsTelemetry.debug("Forward", forward);
            panelsTelemetry.debug("Rotate", rotate);
            panelsTelemetry.debug("Speed Multiplier", SPEED_MULTIPLIER);
            panelsTelemetry.debug("Deadzone", DEADZONE);
        }

        panelsTelemetry.debug("LB Power", leftBackPower);
        panelsTelemetry.debug("LF Power", leftFrontPower);
        panelsTelemetry.debug("RF Power", rightFrontPower);
        panelsTelemetry.debug("RB Power", rightBackPower);

        panelsTelemetry.update(telemetry);

        // Driver Station telemetry
        telemetry.addData("LB Power", leftBackPower);
        telemetry.addData("LF Power", leftFrontPower);
        telemetry.addData("RF Power", rightFrontPower);
        telemetry.addData("RB Power", rightBackPower);

        telemetry.addData("Status", "Running");
        telemetry.addData("Joystick Active", joystickActive);
        telemetry.addData("Speed Multiplier", SPEED_MULTIPLIER);
        telemetry.update();
    }
}
