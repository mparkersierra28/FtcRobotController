package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    //Motors, Servos, etc
    public DcMotor lf, rf, lb, rb;  // drivetrain


    public void init(HardwareMap hw) {
        // Drivetrain
        lf = hw.get(DcMotor.class, "LF");
        rf = hw.get(DcMotor.class, "RF");
        lb = hw.get(DcMotor.class, "LB");
        rb = hw.get(DcMotor.class, "RB");


        // Set directions / modes if needed
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior here
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
