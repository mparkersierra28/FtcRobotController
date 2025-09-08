package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        // Initialize Panels + telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized - Panels Connected");
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
        forward = leftY + rightY;
        forward = Range.clip(forward, -1.0, 1.0);

        // Deadzone check
        joystickActive = Math.abs(strafe) > DEADZONE ||
                Math.abs(forward) > DEADZONE ||
                Math.abs(rotate) > DEADZONE;

        if (joystickActive) {
            // Mecanum formula
            leftBackPower = (forward + strafe - rotate) * SPEED_MULTIPLIER;
            leftFrontPower = (forward - strafe - rotate) * SPEED_MULTIPLIER;
            rightFrontPower = (forward + strafe + rotate) * SPEED_MULTIPLIER;
            rightBackPower = (forward - strafe + rotate) * SPEED_MULTIPLIER;

            // Clip
            leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
            leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
            rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
            rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);

            // Apply to motors
            robot.lb.setPower(leftBackPower);
            robot.lf.setPower(leftFrontPower);
            robot.rf.setPower(rightFrontPower);
            robot.rb.setPower(rightBackPower);
        } else {
            robot.lb.setPower(0);
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);

            leftBackPower = 0;
            leftFrontPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
        }
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

        panelsTelemetry.update();

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
