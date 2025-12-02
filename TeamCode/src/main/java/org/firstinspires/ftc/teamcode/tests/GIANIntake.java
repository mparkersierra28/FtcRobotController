package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test Intake GIAN")
public class GIANIntake extends OpMode {

    private DcMotorEx intakeR, intakeL;

    @Override
    public void init() {
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeLS");
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeRS");
    }

    @Override
    public void loop() {

        intakeL.setPower(-gamepad1.right_trigger);
        intakeR.setPower(gamepad1.right_trigger);

    }
}
