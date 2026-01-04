package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@TeleOp(name = "Tune Servo", group = "Test")
public class TuneServo extends OpMode {
    RobotHardware robot;
    TelemetryManager telemetryM;

    private double servoPos = 0;

    private boolean uPrev = false;
    private boolean dPrev = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down && !dPrev) servoPos -= 0.05;
        if (gamepad1.dpad_up && !uPrev) servoPos += 0.05;
        if (gamepad1.dpad_left) servoPos = 0;
        if (gamepad1.dpad_right) servoPos = 1;
        uPrev = gamepad1.dpad_up;
        dPrev = gamepad1.dpad_down;
        telemetryM.addData("Servo Position", servoPos);
        if (gamepad1.a) {
            robot.gatePush.setPosition(servoPos);
        }
        telemetryM.update(telemetry);
    }
}
