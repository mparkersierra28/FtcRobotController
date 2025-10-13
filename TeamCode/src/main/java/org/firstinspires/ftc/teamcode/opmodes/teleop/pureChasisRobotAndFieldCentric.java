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
import org.firstinspires.ftc.teamcode.software.MecanumDrive;

@Configurable
@TeleOp(name = "Pure Full Chasis", group = "Teleop")
public class pureChasisRobotAndFieldCentric extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    // Panels instance + telemetry
    private TelemetryManager panelsTelemetry;

    // Configurable variables (show up in Panels UI and are live-tunable)
    public static boolean detailedPanels = false;

    public static double SPEED_MULTIPLIER = 1.0;    // 0.1 to 1.0

    public static double DEADZONE = 0.1;
    public static boolean FIELD_CENTRIC = false;

    // Helps create a toggle for A
    boolean aPressed = false;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Initialize Panels + telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Get the driving class
        drive = new MecanumDrive(robot);

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

        // Send telemetry to Panels and DS
        panelsTelemetry = drive.updateTelemetry(panelsTelemetry);
        panelsTelemetry.update(telemetry);
    }
/*
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
    */
}
