package org.firstinspires.ftc.teamcode.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift Hold Test", group = "test")
public class testLifterContinous extends OpMode {

    private DcMotor left;
    private DcMotor right;

    private boolean wasMoving = false;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "linearLift");
        right = hardwareMap.get(DcMotor.class, "Lift_L");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        boolean movingUp = gamepad1.right_bumper;
        boolean movingDown = gamepad1.left_bumper;

        if (movingUp) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            left.setPower(0.3);
            right.setPower(0.3);

            wasMoving = true;

        } else if (movingDown) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            left.setPower(-0.3);
            right.setPower(-0.3);

            wasMoving = true;

        } else {
            // If we were moving and now stopped, lock position
            if (wasMoving) {

                int holdPosition = left.getCurrentPosition();

                left.setTargetPosition(holdPosition);
                right.setTargetPosition(holdPosition);

                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                left.setPower(0.3);
                right.setPower(0.3);

                wasMoving = false;
            }
        }

        telemetry.addData("Left Pos", left.getCurrentPosition());
        telemetry.addData("Right Pos", right.getCurrentPosition());
        telemetry.update();
    }
}
