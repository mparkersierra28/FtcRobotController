package org.firstinspires.ftc.teamcode.hardware.robotConfigurations;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class GregConfig {
    public static void apply(RobotHardware robot, HardwareMap hw) {
        robot.rf = hw.get(DcMotor.class, "greg");
        robot.lf = hw.get(DcMotor.class, "LF");
        robot.lb = hw.get(DcMotor.class, "LB");
        robot.rb = hw.get(DcMotor.class, "RB");
        // === Motor Direction ===
        robot.lf.setDirection(DcMotor.Direction.REVERSE);
        robot.lb.setDirection(DcMotor.Direction.REVERSE);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // === Odometry ===
        robot.odo = hw.get(GoBildaPinpointDriver.class, "odo");
        robot.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        robot.odo.setOffsets(-4.75, -2.0, DistanceUnit.INCH);
        //robot.odo.resetPosAndIMU();

        // Launching stuff
        robot.launcherR = hw.get(DcMotorEx.class, "launcherR");
        robot.launcherL = hw.get(DcMotorEx.class, "launcherL");
        robot.launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.launcherL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(10, 0.7, 0, 0));
        robot.launcherR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(10, 0.7, 0, 0));

        robot.intakeS = hw.get(CRServo.class, "intakeS");
        robot.firstUpS = hw.get(CRServo.class, "firstUpS");
        robot.secondUpS = hw.get(CRServo.class, "secondUpS");
        //robot.thirdUpS = hw.get(CRServo.class, "thirdUpS");
        robot.gatePush = hw.get(Servo.class, "thirdUpS");

        robot.firstUpS.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.secondUpS.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensors
        robot.huskyLens = hw.get(HuskyLens.class, "huskylens");
    }
}
