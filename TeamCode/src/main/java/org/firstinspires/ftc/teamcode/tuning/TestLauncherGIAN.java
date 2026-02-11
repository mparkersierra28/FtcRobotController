package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp (name = "Test GIAN Vel")
public class TestLauncherGIAN extends OpMode {
    private DcMotorEx launcher;
    private DcMotor indexer;

    private boolean aPressed = false;

    private boolean launcherRunning = false;
    boolean upPressedLast = false;
    boolean downPressedLast = false;

    double targetVelocity = 200;          // ticks per second
    double velocityStep = 100;          // how much to change each press

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
    }

    @Override
    public void loop() {
        if (!aPressed && gamepad1.a) {
            launcherRunning = !launcherRunning;
        }
        aPressed = gamepad1.a;

        boolean upPressed = gamepad1.dpad_up;
        boolean downPressed = gamepad1.dpad_down;

        if (upPressed && !upPressedLast) targetVelocity += velocityStep;
        if (downPressed && !downPressedLast) targetVelocity -= velocityStep;

        upPressedLast = upPressed;
        downPressedLast = downPressed;
        if (launcherRunning) launcher.setVelocity(targetVelocity);
        else launcher.setPower(0);

        if (gamepad1.right_bumper) indexer.setPower(-0.5);
        else indexer.setPower(0);
        telemetry.addData("Vel", targetVelocity);
        telemetry.update();
    }
}
