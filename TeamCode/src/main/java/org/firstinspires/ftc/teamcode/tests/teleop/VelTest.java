package org.firstinspires.ftc.teamcode.tests.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;
import org.firstinspires.ftc.teamcode.software.Physics;

@TeleOp(name = "Test Vel", group = "z")
public class VelTest extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    private Physics physics;
    private boolean aPressed = false;
    private boolean lRunning = false;
    private boolean bPressed = false;
    private boolean sRunning = false;
    private double targetVel;

    public static int goalXPos = 140, goalYPos = 4;
    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();
        drive = new MecanumDrive(robot);
        physics = new Physics(robot);
        robot.odo.setPosition(new Pose2D(DistanceUnit.INCH, 120, 22, AngleUnit.RADIANS, 0));

    }

    @Override
    public void loop() {
        if (!aPressed && gamepad1.a) {
            lRunning = !lRunning;
            if (lRunning) {
                robot.odo.update();
                targetVel = physics.getVelocityTpS(physics.getDistanceToPoint(goalXPos, goalYPos));
            }
        }
        aPressed = gamepad1.a;
        if (lRunning) {
            drive.turnInDirection(physics.getHeadingError(goalXPos, goalYPos));
            robot.launcherR.setVelocity(targetVel);
            robot.launcherL.setVelocity(targetVel);
            if (Math.abs(targetVel-robot.launcherR.getVelocity())<=80&&Math.abs(targetVel-robot.launcherL.getVelocity())<=80) {
                robot.thirdUpS.setPower(0.2);
            }
        } else {
                robot.launcherR.setVelocity(0);
                robot.launcherL.setVelocity(0);
                robot.thirdUpS.setPower(0);
            }
        if (!bPressed && gamepad1.b) {
            sRunning = !sRunning;
        }
        bPressed = gamepad1.b;
        if (sRunning) {
            robot.secondUpS.setPower(1);
            robot.firstUpS.setPower(1);
            robot.intakeS.setPower(1);
        } else {
            robot.secondUpS.setPower(0);
            robot.firstUpS.setPower(0);
            robot.intakeS.setPower(0);
        }

        telemetry.addData("Vel", targetVel);

    }
}
