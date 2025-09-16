package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    //Motors, Servos, etc
    public DcMotor lf, rf, lb, rb;  // drivetrain

    public GoBildaPinpointDriver odo;


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

        // Odometry
        odo = hw.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(7.5, 2, DistanceUnit.INCH);
    }
}
