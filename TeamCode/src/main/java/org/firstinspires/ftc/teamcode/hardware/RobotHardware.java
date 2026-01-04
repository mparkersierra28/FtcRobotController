package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.GiveItANameConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.GregConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.HubertConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.PinkyConfig;

import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.GiveItANameConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.GregConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.PinkyConstants;
import org.firstinspires.ftc.teamcode.software.TransferData;

public class RobotHardware {

    // Enum for robot type
    public enum RobotType {
        PINKY,
        GIVE_IT_A_NAME,
        GREG,
        HUBERT
    }
    // Current robot type
    private RobotType robotType;

    // Alliance enum
    public enum Alliance {
        RED,
        BLUE,
        EMPTY
    }
    // Current Alliance
    public Alliance alliance = Alliance.RED;

    private HardwareMap hw;

    // Motors
    public DcMotor lf, rf, lb, rb;

    // Greg
    public DcMotorEx launcherR, launcherL;
    public CRServo intakeS, firstUpS, secondUpS, thirdUpS;
    public Servo gatePush;

    // GIAN
    public DcMotor indexer;
    public DcMotorEx launcher;
    public RevColorSensorV3 intakeSensor;
    //public Servo gateS;
    public CRServo intakeServo;

    // Sensors
    public GoBildaPinpointDriver odo;
    public HuskyLens huskyLens;


    public RobotHardware(HardwareMap hw) {
        this.hw = hw;

        alliance = convertToAlliance(TransferData.getAlliance(hw));

        // Detect robot type based on Right Front name
        detectRobotTypeFromServo();

        if (robotType == null) {
            throw new RuntimeException("Unknown robot: could not detect from Right Front name!");
        }
    }

    private void detectRobotTypeFromServo() {
        try {
            if (hw.get(DcMotor.class, "pinky") != null) {
                robotType = RobotType.PINKY;
                return;
            }
        } catch (Exception ignored) {}
        try {
            if (hw.get(DcMotor.class, "giveitaname") != null) {
                robotType = RobotType.GIVE_IT_A_NAME;
                return;
            }
        } catch (Exception ignored) {}
        try {
            if (hw.get(DcMotor.class, "greg") != null) {
                robotType = RobotType.GREG;
                return;
            }
        } catch (Exception ignored) {}
        try {
            if (hw.get(DcMotor.class, "hubert") != null) {
                robotType = RobotType.HUBERT;
                return;
            }
        } catch (Exception ignored) {}
        // If still not found after timeout
        robotType = null;
    }



    public void setRobotConfig() {
        switch (robotType) {
            case PINKY:
                PinkyConfig.apply(this, hw);
                break;
            case GIVE_IT_A_NAME:
                GiveItANameConfig.apply(this, hw);
                break;
            case GREG:
                GregConfig.apply(this, hw);
                break;
            case HUBERT:
                HubertConfig.apply(this, hw);
                break;
        }
    }

    public Follower setPedroConstants() {
        switch (robotType) {
            case PINKY:
                return PinkyConstants.createFollower(hw);
            case GIVE_IT_A_NAME:
                return GiveItANameConstants.createFollower(hw);
            case GREG:
                return GregConstants.createFollower(hw);
            default:
                throw new RuntimeException("Unknown robot type for PedroPathing: " + robotType);
        }
    }

    public Alliance convertToAlliance(int rawA) {
        if (rawA == 0) return Alliance.RED;
        if (rawA == 1) return Alliance.BLUE;
        return Alliance.EMPTY;
    }
    public boolean isRedAlliance() {
        return alliance.ordinal() == RobotHardware.Alliance.RED.ordinal();
    }
}
