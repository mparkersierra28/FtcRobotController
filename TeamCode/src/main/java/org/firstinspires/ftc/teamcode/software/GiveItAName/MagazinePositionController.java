package org.firstinspires.ftc.teamcode.software.GiveItAName;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Configurable
public class MagazinePositionController {
    private RobotHardware robot;
    private static int[] positions = {0, 260, 520}; // encoder ticks for positions 1, 2, 3
    private int currentIndex; // 0 = position 1, 1 = position 2, 2 = position 3

    public MagazinePositionController(RobotHardware robot) {
        this.robot = robot;
        this.currentIndex = 0; // start at position 1 by default
        robot.indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.indexer.setTargetPosition(positions[currentIndex]);
        robot.indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Move forward in positions (1->2->3->1)
    public void moveForward(double power) {
        int nextIndex = (currentIndex + 1) % positions.length;
        moveToIndex(nextIndex, Math.abs(power));
    }

    // Move backward in positions (1<-2<-3<-1)
    public void moveBackward(double power) {
        int nextIndex = (currentIndex - 1 + positions.length) % positions.length;
        moveToIndex(nextIndex, -Math.abs(power));
    }

    // Internal helper function
    private void moveToIndex(int index, double power) {
        robot.indexer.setTargetPosition(positions[index]);
        robot.indexer.setPower(power);
        currentIndex = index;
    }

    // Optional: check if motor has reached its target
    public boolean isBusy() {
        boolean busy = robot.indexer.isBusy();
        if (!busy) robot.indexer.setPower(0);
        return busy;
    }

    // Optional: get current position index
    public int getCurrentIndex() {
        return currentIndex + 1; // to return 1, 2, 3 instead of 0, 1, 2
    }
}
