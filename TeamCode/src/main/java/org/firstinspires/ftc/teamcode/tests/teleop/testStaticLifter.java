package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "testStaticLifter", group = "test")
public class testStaticLifter extends OpMode {

    private DcMotor left;
    private DcMotor right;

    int targetPosition = 0;

    // For edge detection
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    @Override
    public void init() {

        left = hardwareMap.get(DcMotor.class, "linearLift");
        right = hardwareMap.get(DcMotor.class, "Lift_L");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(0);
        right.setTargetPosition(0);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && !lastDpadUp) {
            targetPosition += 100;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            targetPosition -= 100;
        }

        // Update last states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // Apply target position
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        left.setPower(0.7);
        right.setPower(0.7);

        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current L", left.getCurrentPosition());
        telemetry.addData("Current R", right.getCurrentPosition());
        telemetry.update();
    }
}
