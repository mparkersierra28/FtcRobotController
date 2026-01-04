package org.firstinspires.ftc.teamcode.software.GiveItAName;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Configurable
public class MagazinePositionController {

    private static final int STEP_TICKS = 260;

    private RobotHardware robot;
    private int currentIndex = 0; // 0,1,2 for logical position

    public MagazinePositionController(RobotHardware robot) {
        this.robot = robot;

        robot.indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Always move +260 ticks
    public void moveForward(double power) {
        int currentPos = robot.indexer.getCurrentPosition();
        int target = currentPos + STEP_TICKS;

        robot.indexer.setTargetPosition(target);
        robot.indexer.setPower(Math.abs(power));

        currentIndex = (currentIndex + 1) % 3;
    }

    // Always move -260 ticks
    public void moveBackward(double power) {
        int currentPos = robot.indexer.getCurrentPosition();
        int target = currentPos - STEP_TICKS;

        robot.indexer.setTargetPosition(target);
        robot.indexer.setPower(Math.abs(power));

        currentIndex = (currentIndex - 1 + 3) % 3;
    }

    public boolean isBusy() {
        boolean busy = robot.indexer.isBusy();
        if (!busy) robot.indexer.setPower(0);
        return busy;
    }

    public int getCurrentIndex() {
        return currentIndex + 1; // returns 1–3
    }
}
