package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Double Lift Test", group = "test")
public class LiftTest extends OpMode {
    TelemetryManager telemetryM;
    private DcMotor left;
    private DcMotor right;


    private boolean dPadUpPressed = false;
    private boolean dPadDownPressed = false;





    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "linearLift");
        right = hardwareMap.get(DcMotor.class, "Lift_L");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();




    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            left.setPower(0.5);
            right.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            left.setPower(-0.5);
            right.setPower(-0.5);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
        telemetryM.addData("Pos", left.getCurrentPosition());
        telemetryM.update(telemetry);


    }
}
