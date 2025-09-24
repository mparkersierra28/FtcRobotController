package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Configurable
@TeleOp (name = "Greg Launcher")
public class gregLauncher extends OpMode {
    private RobotHardware robot;

    public double launchPower = 1.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private TelemetryManager telemetryM;


    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.setRobotConfig();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.launcherR.setPower(launchPower);
            robot.launcherL.setPower(launchPower);
        } else if (gamepad1.b) {
            robot.launcherR.setPower(-launchPower);
            robot.launcherL.setPower(-launchPower);
        }

        if (gamepad1.dpad_up && !prevDpadUp) {
            launchPower += 0.05;
        } else if (gamepad1.dpad_down && !prevDpadDown) {
            launchPower -= 0.05;
        }

        // Save current states for next loop
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        // telemetry
        telemetryM.debug("Press A for positive launching motion.\nPress B for negative launching motion");
        telemetryM.debug("Press Up for increased power.\nPress Down for decreased power");
        telemetryM.debug("Current Power: " + launchPower);
        telemetryM.update(telemetry);
    }
}
