package org.firstinspires.ftc.teamcode.pedroPathing.pConstants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class GregConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            //changes with weight
            .mass(9)
            .forwardZeroPowerAcceleration(-33)
            .lateralZeroPowerAcceleration(-57)

            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            //F = friction
            //P = how agressive it returns
            //D = reduces osolations
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0.02))
            //.headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.025))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.09, 0, 0.00001, 0.6, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("LF")
            .leftRearMotorName("LB")
            .rightFrontMotorName("greg")
            .rightRearMotorName("RB")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66) //changes with weight
            .yVelocity(56); //Changes with weight

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.75)
            .strafePodX(-2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //.customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1.125,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
