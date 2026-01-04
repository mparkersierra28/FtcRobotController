package org.firstinspires.ftc.teamcode.software.GiveItAName;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Sorter {

    public enum Obj {
        EMPTY,
        PURPLE,
        GREEN
    }

    private  RobotHardware robot;
    private MagazinePositionController magazine;

    // 0 = INTAKE, 1 = WAITING, 2 = EXIT
    private Obj[] slots = { Obj.EMPTY, Obj.EMPTY, Obj.EMPTY };

    // ---------------- PATTERN SYSTEM ----------------

    // Pattern table: pattern[i][step]
    private final Obj[][] patterns = {
            { Obj.GREEN,  Obj.PURPLE, Obj.PURPLE }, // Pattern 1
            { Obj.PURPLE, Obj.PURPLE, Obj.GREEN  }, // Pattern 2
            { Obj.PURPLE, Obj.GREEN,  Obj.PURPLE }  // Pattern 3
    };

    private int currentPattern = 0;  // 0,1,2 = pattern 1..3
    private int patternStep = 0;     // 0,1,2 index into pattern

    // Constructor
    public Sorter(RobotHardware robot, MagazinePositionController magazine) {
        this.robot = robot;
        this.magazine = magazine;
    }

    /**
     * Set which of the 3 patterns to use.
     */
    public void setPattern(int patternNumber) {
        if (patternNumber < 0 || patternNumber > 2) return;
        currentPattern = patternNumber;
        patternStep = 0; // reset step
    }

    /**
     * Move to the next item in the pattern (cyclic 0→1→2→0)
     */
    public void advancePattern() {
        patternStep = (patternStep + 1) % 3;
    }

    /**
     * Get expected color for the current step of the pattern.
     */
    public Obj getExpectedColor() {
        return patterns[currentPattern][patternStep];
    }

    // ---------------- ROTATION SYSTEM ----------------

    public void rotateForward(double motorPower) {
        // Rotate logical slots
        Obj temp = slots[2];
        slots[2] = slots[1];
        slots[1] = slots[0];
        slots[0] = temp;

        // Move physical magazine
        magazine.moveForward(motorPower);
    }

    public void rotateBackward(double motorPower) {
        Obj temp = slots[0];
        slots[0] = slots[1];
        slots[1] = slots[2];
        slots[2] = temp;

        magazine.moveBackward(motorPower);
    }


    // ---------------- SAFE PLACEMENT ----------------

    /**
     * Place into a slot only if empty.
     * @return true if placed, false if slot already occupied
     */
    public boolean placeIfEmpty(Obj obj, int index) {
        if (slots[index] == Obj.EMPTY) {
            slots[index] = obj;
            return true;
        }
        return false;
    }

    // ---------------- COLOR DETECTION ----------------

    private Obj detectColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Very simple thresholds – tune these in telemetry
        if (g > r && g > b && g > 90)   return Obj.GREEN;
        //37, 120, 102
        if (b > 100 && r > 30) return Obj.PURPLE;
        //60, 88, 133

//        if (r < 40 && g < 40 && b < 40) return Obj.EMPTY;
        //23, 46, 38

        return Obj.EMPTY;
    }

    /**
     * Detect at intake sensor.
     * If no ball → returns false.
     * If color → puts it into slot 0 and returns true.
     */
    public boolean detectIntake() {
        Obj detected = detectColor(robot.intakeSensor);

        if (detected == Obj.EMPTY) {
            return false;
        }



        slots[0] = detected;
        return true;
    }


//    /**
//     * Updates EXIT slot from the exit sensor and verifies it is empty.
//     * Also advances the pattern
//     * @return true if the exit is empty, false if there is a ball
//     */
//    public boolean checkAndClearExit() {
//        Obj sensed = detectColor(robot.exitSensor);
//        slots[2] = sensed;
//
//        // If sensed is a ball, exit is NOT empty
//        if (sensed == Obj.GREEN || sensed == Obj.PURPLE) {
//            return false;
//        }
//        advancePattern();
//        // Otherwise it's empty (slot already set to sensed)
//        return true;
//    }

    /**
     * Rotates the slots (and magazine) so that an EMPTY slot
     * ends up at the INTAKE position (slot 0), taking the shortest path.
     * @param motorPower Power to use for the magazine motor rotation
     * @return true if an EMPTY slot was found and moved to intake, false if no EMPTY slots exist
     */
    public boolean moveEmptyToIntake(double motorPower) {
        int emptyIndex = -1;
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == Obj.EMPTY) {
                emptyIndex = i;
                break;
            }
        }

        if (emptyIndex == -1) return false;

        int steps = forwardSteps(emptyIndex, 0);

        for (int i = 0; i < steps; i++) {
            rotateForward(motorPower);
        }

        return true;
    }


    /**
     * Rotates the sorter so that the object matching the next pattern step
     * ends up at the EXIT position (slot 2).
     * If that color is not present, falls back to any non-empty object.
     * Returns false if all slots are empty.
     *
     * @param motorPower Power to move the magazine
     * @return true if a rotation was done, false if everything is empty
     */
    public boolean moveNextPatternToExit(double motorPower) {
        Obj target = getExpectedColor();

        int targetIndex = -1;
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == target) {
                targetIndex = i;
                break;
            }
        }

        if (targetIndex == -1) {
            for (int i = 0; i < slots.length; i++) {
                if (slots[i] != Obj.EMPTY) {
                    targetIndex = i;
                    break;
                }
            }
        }

        if (targetIndex == -1) return false;

        int steps = forwardSteps(targetIndex, 2);

        for (int i = 0; i < steps; i++) {
            rotateForward(motorPower);
        }

        return true;
    }

    private int forwardSteps(int from, int to) {
        return (to - from + 3) % 3;
    }

    public void shootBall(double motorPower) {
        slots[2] = Obj.EMPTY;
        rotateBackward(motorPower);
        advancePattern();
    }



    // ---------------- GETTERS ----------------

    public Obj getIntake()  { return slots[0]; }
    public Obj getWaiting() { return slots[1]; }
    public Obj getExit()    { return slots[2]; }
}
