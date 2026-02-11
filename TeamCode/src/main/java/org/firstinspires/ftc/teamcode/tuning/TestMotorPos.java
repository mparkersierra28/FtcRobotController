package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Reset Linear", group = "2")
public class TestMotorPos extends OpMode {

    public TelemetryManager telemetryM;

    private DcMotor magazine;

    @Override
    public void init() {
        magazine = hardwareMap.get(DcMotor.class, "linearLift");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            magazine.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            magazine.setPower(-0.5);
        } else magazine.setPower(0);
        telemetryM.addData("MotorPos", magazine.getCurrentPosition());
        telemetryM.update(telemetry);
    }
}
