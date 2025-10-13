package org.firstinspires.ftc.teamcode.hardware.robotConfigurations;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class PinkyConfig {
    public static void apply(RobotHardware robot, HardwareMap hw) {
        robot.rf = hw.get(DcMotor.class, "pinky");
        // === Motor Direction ===
        robot.rf.setDirection(DcMotor.Direction.REVERSE);
        robot.rb.setDirection(DcMotor.Direction.REVERSE);


        // === Odometry ===
        robot.odo = hw.get(GoBildaPinpointDriver.class, "odo");
        robot.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        robot.odo.setOffsets(-7.5, -7.5, DistanceUnit.INCH);
    }
}
