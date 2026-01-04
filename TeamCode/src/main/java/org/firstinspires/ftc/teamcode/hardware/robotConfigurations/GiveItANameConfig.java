package org.firstinspires.ftc.teamcode.hardware.robotConfigurations;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class GiveItANameConfig {
    public static void apply(RobotHardware robot, HardwareMap hw) {
        robot.rf = hw.get(DcMotor.class, "giveitaname");
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

        robot.indexer = hw.get(DcMotor.class, "indexer");

        robot.launcher = hw.get(DcMotorEx.class, "launcher");


        // === Odometry ===
        robot.odo = hw.get(GoBildaPinpointDriver.class, "odo");
        robot.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        robot.odo.setOffsets(7.5, 2, DistanceUnit.INCH);

        //robot.huskyLens = hw.get(HuskyLens.class, "huskylens");

        robot.intakeSensor = hw.get(RevColorSensorV3.class, "intakeSensor");
        //robot.exitSensor = hw.get(RevColorSensorV3.class, "exitSensor");

        robot.intakeServo = hw.get(CRServo.class, "intake");
    }
}
