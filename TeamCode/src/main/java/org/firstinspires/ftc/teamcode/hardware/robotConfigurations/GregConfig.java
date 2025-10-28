package org.firstinspires.ftc.teamcode.hardware.robotConfigurations;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        robot.launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.intakeS = hw.get(CRServo.class, "intakeS");
        robot.firstUpS = hw.get(CRServo.class, "firstUpS");
        robot.secondUpS = hw.get(CRServo.class, "secondUpS");
        robot.thirdUpS = hw.get(CRServo.class, "thirdUpS");

        robot.firstUpS.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.secondUpS.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
