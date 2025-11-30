package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.CameraQR;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;
import org.firstinspires.ftc.teamcode.software.Physics;
@Configurable
@TeleOp(name="Test Camera QR", group="Utilities")
public class TestCameraQR extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    private CameraQR qrReader;
    private Physics physics;
    private TelemetryManager telemetryM;

    public static double k = 0.8; // Probably between 0.6 to 0.95

    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean lbPressed = false;
    private boolean rbPressed = false;
    private double horizontalDis = 0;
    private double qrDir = 0;
    private double qrAngle = 0;




    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        drive = new MecanumDrive(robot);

        qrReader = new CameraQR(robot);

        physics = new Physics(robot);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void loop() {

        if (gamepad1.a && !aPressed) {
            horizontalDis = qrReader.getHorizontalDis();
            aPressed = true;
        } else if (!gamepad1.a) aPressed = false;

        if (gamepad1.b && !bPressed) {
            qrDir = qrReader.getQRDir();
            bPressed = true;
        } else if (!gamepad1.b) bPressed = false;

        if (gamepad1.x && !xPressed) {
            qrAngle = qrReader.getTargetAngleDeg();
            xPressed = true;
        } else if (!gamepad1.x) xPressed = false;

        if (gamepad1.left_bumper) {
            drive.turnToAngle(qrReader.getTargetAngleDeg());
        } else drive.stopMotors();
        double v = 0;
        if (gamepad1.right_bumper) {
            v = physics.getVelocityTpS(144, 0);
            if (v!=-1){
                robot.launcherL.setVelocity(v/k);
                robot.launcherR.setVelocity(v/k);
            }


        } else {
            robot.launcherL.setPower(0);
            robot.launcherR.setPower(0);
        }


        telemetryM.debug("Horizontal Dis press a", horizontalDis);
        telemetryM.debug("QR dir press x", qrDir);
        telemetryM.debug("QR angle press x", qrAngle);
        telemetryM.debug("Estimated Velocity", v);
        telemetryM.debug("K", k);
        telemetryM.update(telemetry);
    }
}
