package org.firstinspires.ftc.teamcode.software;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShootingPhysics {

    /*** --- Physical constants --- ***/
    private static final double G = 9.80665; // gravity (m/s^2)

    /*** --- Geometry: fixed robot + field measurements --- ***/
    // Camera → Launcher offsets
    public static double dxCamToLauncher = 0.10;  // forward offset (m)
    public static double dzCamToLauncher = 0.05;  // launcher is above camera (m)
    public static double thetaCamToLauncher = Math.toRadians(10); // launcher pitched +10° relative to camera

    // QR → Basket offsets
    public static double dxQrToBasket = 0.15;  // basket is behind QR (m)
    public static double dzQrToBasket = 0.25;  // basket is above QR (m)

    // Basket height above floor (absolute, optional if using relative values)
    public static double basketHeight = 1.20; // m (top/right-angle point)

    // Camera height above floor (used if absolute positions are preferred)
    public static double cameraHeight = 0.90; // m

    /*** --- Launcher + tuning parameters --- ***/
    public static double margin = 0.03;        // aim slightly low (m)
    public static double descendSafety = 0.95; // ensure descending arc
    public static double k_v = 1.0;            // scale correction for real-world differences
    public static double v_bias = 0.0;         // small additive bias (m/s)
    public static double k_cmd = 1.0;          // scale factor from velocity to motor command
    public static double b_cmd = 0.0;          // offset for motor command

    /*** --- Constructors --- ***/
    public ShootingPhysics() {}

    /*** --- Core method: compute required launch velocity --- ***/
    public double getRequiredVelocity(double alpha, double x_cam) {
        // alpha: camera pitch angle to QR (radians)
        // x_cam: horizontal distance from camera to QR (m)

        // Step 1: Horizontal distance from launcher to basket
        double x = x_cam + dxCamToLauncher + dxQrToBasket;

        // Step 2: Vertical difference from launcher to basket
        double h_launcher = cameraHeight + dzCamToLauncher;
        double h_qr = basketHeight - dzQrToBasket;
        double h = (basketHeight) - h_launcher;

        // Step 3: Launcher pitch angle
        double theta = alpha + thetaCamToLauncher;

        // Step 4: Target vertical offset (include margin)
        double h_target = h - margin;

        // Step 5: Physics equation (no drag)
        double denom = 2 * (x * Math.tan(theta) - h_target);
        if (denom <= 0) {
            System.out.println("Target unreachable at this angle!");
            return Double.NaN;
        }

        double v_raw = (x / Math.cos(theta)) * Math.sqrt(G / denom);

        // Step 6: Apply scale & bias
        double v_final = k_v * v_raw + v_bias;

        // Step 7: Optional descent check
        if (!isDescending(v_final, x, theta)) {
            System.out.println("Warning: projectile not descending at target!");
        }

        return v_final;
    }

    /*** --- Helper: check if projectile is descending at distance x --- ***/
    private boolean isDescending(double v, double x, double theta) {
        double vy = v * Math.sin(theta) - (G * x) / (v * Math.cos(theta));
        return vy < 0; // true if descending
    }

    /*** --- Helper: map velocity to motor command (e.g., RPM setpoint) --- ***/
    public double velocityToMotorCommand(double v_final) {
        return k_cmd * v_final + b_cmd;
    }

}
