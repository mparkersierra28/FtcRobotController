package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.GiveItANameConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.GregConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.HubertConfig;
import org.firstinspires.ftc.teamcode.hardware.robotConfigurations.PinkyConfig;

import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.GiveItANameConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.GregConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pConstants.PinkyConstants;

public class RobotHardware {

    // Enum for robot type
    public enum RobotType {
        PINKY,
        GIVE_IT_A_NAME,
        GREG,
        HUBERT
    }

    private HardwareMap hw;

    // Motors
    public DcMotor lf, rf, lb, rb, launcherR, launcherL;
    public CRServo intakeS, firstUpS, secondUpS, thirdUpS;
    public GoBildaPinpointDriver odo;

    // Current robot type
    private RobotType robotType;

    public RobotHardware(HardwareMap hw) {
        this.hw = hw;

        // Detect robot type based on servo name
        detectRobotTypeFromServo();
        if (robotType == null) {
            throw new RuntimeException("Unknown robot: could not detect from servo name!");
        }

        // Initialize motors
        // RF is initialized inside of Configs
        lf = hw.get(DcMotor.class, "LF");
        lb = hw.get(DcMotor.class, "LB");
        rb = hw.get(DcMotor.class, "RB");

        // Apply robot-specific configuration
        setRobotConfig();

        // Universal motor settings
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
}
