package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Velocity PIDF Tuner", group = "Tuning")
public class VelocityPIDFTuner extends OpMode {

    private DcMotorEx motor;
    private TelemetryManager telemetryM;
    private ElapsedTime timer = new ElapsedTime();

    // --- PIDF Coefficients (editable via Panels) ---
    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // --- Target velocity (ticks per second) ---
    public static double targetVelocity = 1000.0;

    // --- Telemetry refresh timer ---
    private double lastUpdate = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "launcherL"); // rename if needed
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Apply initial PIDF
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug(hardwareMap.get(DcMotorEx.class, "launcherR").getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetryM.debug("P", kP);
        telemetryM.debug("I", kI);
        telemetryM.debug("D", kD);
        telemetryM.debug("F", kF);
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {

        // Set target velocity
        motor.setVelocity(targetVelocity);

        // Read actual velocity (ticks per second)
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        // Update telemetry once every 0.1 seconds
        if (timer.seconds() - lastUpdate > 0.1) {
            lastUpdate = timer.seconds();


            telemetryM.addData("Target (ticks/s)", targetVelocity);
            telemetryM.addData("Current (ticks/s)", currentVelocity);
            telemetryM.addData("Error", error);
            telemetryM.addData("Power", motor.getPower());
            telemetryM.update(telemetry);
        }
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
