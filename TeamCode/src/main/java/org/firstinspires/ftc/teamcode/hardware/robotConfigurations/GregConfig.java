package org.firstinspires.ftc.teamcode.hardware.robotConfigurations;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class GregConfig {
    public static void apply(RobotHardware robot, HardwareMap hw) {
        robot.rf = hw.get(DcMotor.class, "greg");
        // === Motor Direction ===
        robot.lf.setDirection(DcMotor.Direction.REVERSE);
        robot.lb.setDirection(DcMotor.Direction.REVERSE);


        // === Odometry ===
        robot.odo = hw.get(GoBildaPinpointDriver.class, "odo");
        robot.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        robot.odo.setOffsets(-4.5, 2.5, DistanceUnit.INCH);
        robot.odo.resetPosAndIMU();

        // Launching stuff
        robot.launcherR = hw.get(DcMotor.class, "launcherR");
        robot.launcherL = hw.get(DcMotor.class, "launcherL");
        robot.launcherL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.lastS = hw.get(CRServo.class, "lastS");
        robot.intakeS = hw.get(CRServo.class, "intakeS");
        robot.lastS.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakeS.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
