package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Locale;

@TeleOp(name="Odometer Test", group="Diagnostics")
public class testOdometry extends OpMode {

    private RobotHardware robot;
    private TelemetryManager panels;

    // last positions to calculate deltas
    private double lastX = 0, lastY = 0, lastHeading = 0;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);

        panels = PanelsTelemetry.INSTANCE.getTelemetry();;

        telemetry.addLine("Odometer Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update values, always call once per loop
        robot.odo.update();
        Pose2D robotPosition = robot.odo.getPosition();
        // get current values from odo
        double x = robotPosition.getX(DistanceUnit.INCH);           // inches (or cm if that’s your units)
        double y = robotPosition.getY(DistanceUnit.INCH);
        double heading = robotPosition.getHeading(AngleUnit.DEGREES); // degrees or radians depending on your odo

        // deltas since last loop
        double dx = x - lastX;
        double dy = y - lastY;
        double dHeading = heading - lastHeading;

        String direction = "Stationary";
        if (Math.abs(dx) > Math.abs(dy) && Math.abs(dx) > Math.abs(dHeading)) {
            direction = dx > 0 ? "Right" : "Left";
        } else if (Math.abs(dy) > Math.abs(dx) && Math.abs(dy) > Math.abs(dHeading)) {
            direction = dy > 0 ? "Forward" : "Backward";
        } else if (Math.abs(dHeading) > 0.5) { // small threshold
            direction = dHeading > 0 ? "Turning Left (CCW)" : "Turning Right (CW)";
        }

        // TELEMETRY
        telemetry.addData("X", "%.2f", x);
        telemetry.addData("Y", "%.2f", y);
        telemetry.addData("Heading", "%.2f", heading);
        telemetry.addData("ΔX / ΔY / ΔH", "%.2f / %.2f / %.2f", dx, dy, dHeading);
        telemetry.addData("Direction", direction);
        telemetry.update();

        // PANELS TELEMETRY
        panels.debug("X", String.format(Locale.US, "%.2f", x));
        panels.debug("Y", String.format(Locale.US, "%.2f", y));
        panels.debug("Heading", String.format(Locale.US, "%.2f", heading));
        panels.debug("ΔX", String.format(Locale.US, "%.2f", dx));
        panels.debug("ΔY", String.format(Locale.US, "%.2f", dy));
        panels.debug("ΔH", String.format(Locale.US, "%.2f", dHeading));
        panels.debug("Direction", direction);
        panels.update();

        // update last values
        lastX = x;
        lastY = y;
        lastHeading = heading;
    }
}
