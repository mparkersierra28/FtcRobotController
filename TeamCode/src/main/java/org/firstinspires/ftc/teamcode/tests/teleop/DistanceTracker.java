package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name="Distance Tracker", group="Tests")
public class DistanceTracker extends OpMode {

    private Follower follower;
    private Pose startPose;

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        // Initialize PedroPathing follower
        follower = robot.setPedroConstants();

        // Save the starting pose as the "zero" reference
        startPose = follower.getPose();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        // Update follower each loop
        follower.update();

        // Current pose
        Pose currentPose = follower.getPose();

        // Differences from the reset point
        double dx = currentPose.getX() - startPose.getX();
        double dy = currentPose.getY() - startPose.getY();
        double dHeading = Math.toDegrees(currentPose.getHeading() - startPose.getHeading());

        // Send to telemetry
        telemetryM.debug("ΔX", "%.2f", dx);
        telemetryM.debug("ΔY", "%.2f", dy);
        telemetryM.debug("ΔRot (deg)", "%.2f", dHeading);
        telemetryM.update(telemetry);

        // Reset origin when pressing A
        if (gamepad1.a) {
            startPose = currentPose;
        }
    }
}
