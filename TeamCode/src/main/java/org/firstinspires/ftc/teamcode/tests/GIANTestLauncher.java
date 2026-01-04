package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Launcher", group = "Tune")
public class GIANTestLauncher extends OpMode {

    public TelemetryManager telemetryM;

    private DcMotor magazine;
    private DcMotor launcher;

    private boolean aPressed = false;
    private double lPower = 0;

    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;

    @Override
    public void init() {
        magazine = hardwareMap.get(DcMotor.class, "indexer");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        magazine.setTargetPosition(0);
//        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) magazine.setPower(-0.5);
        else if (gamepad1.right_bumper) magazine.setPower(0.5);
        if (!aPressed && gamepad1.a) {
            launcher.setPower(lPower);
        }
        aPressed = gamepad1.a;

        if (gamepad1.dpad_up && !dpadUpLast) {
            lPower += 0.1;
        }

        // D-pad DOWN → decrease power
        if (gamepad1.dpad_down && !dpadDownLast) {
            lPower -= 0.1;
        }

        // update debounce states
        dpadUpLast = gamepad1.dpad_up;
        dpadDownLast = gamepad1.dpad_down;

        // clamp power
        lPower = Math.max(-1.0, Math.min(1.0, lPower));

        telemetryM.debug(lPower);
        telemetryM.update(telemetry);
    }
}
