package org.firstinspires.ftc.teamcode.software;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class CameraQR {
    RobotHardware robot;

    // (0,0) is at upper left corner
    public static int hxCenter = 160;
    public static int hyCenter = 120;

    private int patternID = 0;
    public static int blueID = 1;
    public static int redID = 2;
    public static int margin = 10; // deadzone margin in pixels

    // Physical parameters
    public static double qrRealSize = 0.10; // cm, actual QR width
    public static double focalLengthPx = 800; // focal length of camera in pixels
    public static double cameraPitchDeg = 20; // camera pitch in degrees

    public CameraQR(RobotHardware rob) {
        robot = rob;
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        patternID = robot.alliance == RobotHardware.Alliance.RED ? redID : blueID;
    }

    /**
     * Returns the closest valid QR block detected by the HuskyLens.
     */
    private HuskyLens.Block getClosestQRBlock() {
        HuskyLens.Block[] blocks = robot.huskyLens.blocks();
        if (blocks == null || blocks.length == 0) return null;

        HuskyLens.Block closest = null;
        for (HuskyLens.Block block : blocks) {
            if (block == null) continue;
            if (closest == null || block.height > closest.height) {
                closest = block;
            }
            if (block.id == patternID) closest = block;
        }
        return closest;
    }

    /**
     * Converts block coordinates into camera space (Xc, Yc, Zc).
     */
    private double[] getCameraCoordinates(HuskyLens.Block block) {
        double Zc = focalLengthPx * qrRealSize / block.width;
        double Xc = (block.x - hxCenter) / focalLengthPx * Zc;
        double Yc = (block.y - hyCenter) / focalLengthPx * Zc;
        return new double[]{Xc, Yc, Zc};
    }

    /**
     * Normalizes an angle (radians) to (-π, π).
     */
    private double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    /**
     * Gets the direction of the QR relative to camera center.
     * @return 1 if to the left (CCW), -1 if to the right (CW), 0 if centered
     */
    public double getQRDir() {
        HuskyLens.Block block = getClosestQRBlock();
        if (block == null) return 0;

        int dx = block.x - hxCenter;
        if (Math.abs(dx) <= margin) return 0;
        return dx > 0 ? 1 : -1;
    }

    /**
     * Calculates the horizontal distance to the QR code (in cm).
     */
    public double getHorizontalDis() {
        HuskyLens.Block block = getClosestQRBlock();
        if (block == null) return -1;

        double[] coords = getCameraCoordinates(block);
        double Xc = coords[0], Yc = coords[1], Zc = coords[2];

        // rotate by camera pitch
        double theta = Math.toRadians(cameraPitchDeg);
        double Zw = Math.sin(theta) * Yc + Math.cos(theta) * Zc;
        double Xw = Xc;

        return Math.sqrt(Xw * Xw + Zw * Zw);
    }

    /**
     * Calculates the exact angle (in degrees) the robot must face to look directly at the QR.
     */
    public double getTargetAngleDeg() {
        HuskyLens.Block block = getClosestQRBlock();
        if (block == null) return -1;

        double[] coords = getCameraCoordinates(block);
        double Xc = coords[0], Zc = coords[2];

        // camera angle offset
        double angleOffset = Math.atan2(Xc, Zc);

        // robot heading
        robot.odo.update();
        double robotHeadingRad = robot.odo.getPosition().getHeading(AngleUnit.RADIANS);

        // combine and normalize
        double targetHeadingRad = normalizeAngle(robotHeadingRad + angleOffset);
        return Math.toDegrees(targetHeadingRad);
    }
}
