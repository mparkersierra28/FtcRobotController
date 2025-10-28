package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
}
