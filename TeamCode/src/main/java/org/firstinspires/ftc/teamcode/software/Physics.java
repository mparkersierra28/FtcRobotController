package org.firstinspires.ftc.teamcode.software;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
public class Physics {
    private RobotHardware robot;
    private CameraQR camera;

    public static double lAngle = Math.toRadians(50);
    public static double targetHeight = 50;
    public static double targety = 110;
    public static double dragconst = 0.00109;
    // Gravity in cm/s^2
    private final double g = 980.665; // approximate gravity in cm/s^2

    public static double k = 1.25;
    public Physics(RobotHardware robot) {
        this.robot = robot;
        this.camera = new CameraQR(robot);
    }

    /**
     * Calculates the required initial velocity (u) to hit a target.
     * @return velocity in cm/s
     */
    public double getVelocityTpS(double targetX, double targetY) {
        // It is in Inches
        robot.odo.update();
        // Get horizontal distance to QR
        double x = getDistanceToPoint(targetX, targetY);

        if (x < 40) return -1; // no valid target / too close to risk it

        // Projectile equation: y = tan(q) * x - (g x^2) / (2 u^2 cos^2(q))
        // Solve for u:
        double denominator = Math.tan(lAngle) * x - targetHeight;
        if (denominator <= 0) return -1; // physically impossible

        double u = x * Math.sqrt(g / (2 * Math.pow(Math.cos(lAngle), 2) * denominator));

        // velocity (cm/s)/circumference(0.096 * PI)
        u = (u / (0.096)) * 28 * k;

        return u;
    }

    /**
     * Calculates vel using drag
     * @param targetX how far away the goal x is
     * @return vel for
     */
    public double getVelocityTpS(double targetX) {
        double alphafunction = Math.exp((dragconst*targetX)/Math.cos(lAngle));

        double launchvelnumerator = g*((alphafunction - 1)*(alphafunction - 1));
        double launchveldenominator = 2*dragconst*dragconst*(30.48 + (Math.sin(lAngle)/dragconst)*Math.log(alphafunction)-1.1);
        if (launchveldenominator <= 0) return -1;
        return((Math.sqrt(launchvelnumerator/launchveldenominator))/30.159)*28 * k;
    }
    /**
     * Returns the angle (in radians) from the robot's current odometry position
     * to a target coordinate in YOUR coordinate system:
     *
     *  X axis = forward
     *  Y axis = left
     *
     * Angle returned is field-centric, where:
     *  0 rad = facing forward (positive X)
     *  + rad = turning left (counterclockwise)
     */
    public double getAngleToPoint(double targetX, double targetY) {
        robot.odo.update();
        double robotX = robot.odo.getPosX(DistanceUnit.INCH);  // in
        double robotY = robot.odo.getPosY(DistanceUnit.INCH);  // in

        double dx = targetX - robotX;  // forward difference
        double dy = targetY - robotY;  // left difference

        // angle = atan2(left, forward)
        return Math.atan2(dy, dx);
    }

    /**
     * Returns how much the robot must rotate (radians) to face the target coordinate.
     * Positive = turn left, Negative = turn right.
     */
    public double getHeadingError(double targetX, double targetY) {
        double targetAngle = getAngleToPoint(targetX, targetY);
        double robotHeading = robot.odo.getHeading(AngleUnit.RADIANS);  // radians, same coordinate frame

        double error = targetAngle - robotHeading;

        // Normalize to (-π, π)
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }
    /**
     * Returns the horizontal distance (in cm) from the robot's current odometry
     * position to a target coordinate.
     *
     * Coordinate system:
     *   X = forward
     *   Y = left
     */
    public double getDistanceToPoint(double targetX, double targetY) {
        double robotX = robot.odo.getPosX(DistanceUnit.CM);
        double robotY = robot.odo.getPosY(DistanceUnit.CM);

        double dx = targetX - robotX;  // forward distance
        double dy = targetY - robotY;  // left distance

        return Math.sqrt(dx * dx + dy * dy);
    }


}
