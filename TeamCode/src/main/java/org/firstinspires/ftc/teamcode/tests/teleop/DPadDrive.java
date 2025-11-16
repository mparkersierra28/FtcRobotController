package org.firstinspires.ftc.teamcode.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name="Simple DPad Drive", group="Example")
public class DPadDrive extends OpMode {
    RobotHardware robot;

    double speed = 0.2;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        double y = 0, x = 0, r = 0;

        if (gamepad1.dpad_up) y = -1;
        else if (gamepad1.dpad_down) y = 1;

        if (gamepad1.dpad_left) x = -1;
        else if (gamepad1.dpad_right) x = 1;

        if (gamepad1.left_bumper) r = -1;
        else if (gamepad1.right_bumper) r = 1;

        double LFp = y + x + r;
        double RFp = y - x - r;
        double LBp = y - x + r;
        double RBp = y + x - r;

        robot.lf.setPower(LFp * speed);
        robot.rf.setPower(RFp * speed);
        robot.lb.setPower(LBp * speed);
        robot.rb.setPower(RBp * speed);
    }
}
