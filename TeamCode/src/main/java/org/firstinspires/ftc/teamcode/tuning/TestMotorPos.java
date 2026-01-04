package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Test Motor Pos", group = "Tune")
public class TestMotorPos extends OpMode {

    public TelemetryManager telemetryM;

    private DcMotor magazine;

    @Override
    public void init() {
        magazine = hardwareMap.get(DcMotor.class, "indexer");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetryM.addData("MotorPos", magazine.getCurrentPosition());
        telemetryM.update(telemetry);
    }
}
