package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TestColorSensor extends OpMode {
    private RevColorSensorV3 sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensorExit");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addLine(String.valueOf(sensor.red()));
            telemetry.addLine(String.valueOf(sensor.green()));
            telemetry.addLine(String.valueOf(sensor.blue()));
        }
        telemetry.update();
    }
}
