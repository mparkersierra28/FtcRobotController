package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "Test Intake GIAN")
public class GIANIntake extends OpMode {

    private CRServo intakeR, intakeL;

    @Override
    public void init() {
        intakeL = hardwareMap.get(CRServo.class, "intakeLS");
    }

    @Override
    public void loop() {

        intakeL.setPower(-gamepad1.right_trigger);

    }
}
