package org.firstinspires.ftc.teamcode.software;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
public class Physics {
    private RobotHardware robot;
    private CameraQR camera;

    public static double lAngle = Math.toRadians(50);
    public static double targetHeight = 100;

    // Gravity in cm/s^2
    private static final double g = 980.665; // approximate gravity in cm/s^2

    public Physics(RobotHardware robot) {
        this.robot = robot;
        this.camera = new CameraQR(robot);
    }

    /**
     * Calculates the required initial velocity (u) to hit a target.
     * @return velocity in cm/s
     */
    public double getVelocity() {
        // Get horizontal distance to QR
        double x = camera.getHorizontalDis();
        if (x < 1) return -1; // no valid target / too close to risk it

        // Projectile equation: y = tan(q) * x - (g x^2) / (2 u^2 cos^2(q))
        // Solve for u:
        double denominator = Math.tan(lAngle) * x - targetHeight;
        if (denominator <= 0) return -1; // physically impossible

        double u = x * Math.sqrt(g / (2 * Math.pow(Math.cos(lAngle), 2) * denominator));
        return u;
    }
}
