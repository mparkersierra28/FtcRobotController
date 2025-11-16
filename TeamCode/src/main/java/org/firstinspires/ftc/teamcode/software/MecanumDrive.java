package org.firstinspires.ftc.teamcode.software;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
@Configurable
public class MecanumDrive {

    private final RobotHardware robot;

    // Configurable fields
    public final static double DEADZONE = 0.05;
    public double speed = 1.0;
    public static boolean detailedPanels;

    // State variables
    private boolean FIELD_CENTRIC = false;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double strafe, forward, rotate;
    private boolean joystickActive;

    // Constructors
    public MecanumDrive(RobotHardware robot) {
        this.robot = robot;
        //this.robot.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, ((this.robot.alliance  == RobotHardware.Alliance.RED) ? 90 : -90)));
    }

    // Allows teleop to adjust drive mode or speed multiplier
    public void setFieldCentric(boolean fieldCentric) {
        this.FIELD_CENTRIC = fieldCentric;
    }

    // Main update method (called each loop)
    /**
     * Uses stick inputs for drive train, uses centric defined in init
     * */
    public void update(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        strafe = leftStickX;
        rotate = rightStickX;

        forward = -(leftStickY + rightStickY);
        forward = Range.clip(forward, -1.0, 1.0);

        joystickActive = Math.abs(strafe) > DEADZONE ||
                Math.abs(forward) > DEADZONE ||
                Math.abs(rotate) > DEADZONE;

        if (joystickActive) {
            if (FIELD_CENTRIC) {
                fieldCentricDrive(strafe, forward, rotate);
            } else {
                robotCentricDrive(strafe, forward, rotate);
            }
        } else {
            stopMotors();
        }
    }

    // --- Robot Centric ---
    private void robotCentricDrive(double strafe, double forward, double rotate) {
        leftBackPower   = (forward - strafe + rotate);
        leftFrontPower  = (forward + strafe + rotate);
        rightFrontPower = (forward - strafe - rotate);
        rightBackPower  = (forward + strafe - rotate);

        applyMotorPowers();
    }

    // --- Field Centric ---
    private void fieldCentricDrive(double strafe, double forward, double rotate) {
        robot.odo.update();
        Pose2D robotPosition = robot.odo.getPosition();
        double headingRad = robotPosition.getHeading(AngleUnit.RADIANS);

        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double x = strafe * cosA - forward * sinA;
        double y = strafe * sinA + forward * cosA;

        leftFrontPower  = (y + x + rotate);
        rightFrontPower = (y - x - rotate);
        leftBackPower   = (y - x + rotate);
        rightBackPower  = (y + x - rotate);

        applyMotorPowers();
    }

    // Rotates the robot to a target heading (degrees)
    public void turnToAngle(double targetAngleDeg) {
        // Normalize target
        targetAngleDeg = (targetAngleDeg % 360 + 360) % 360;

        robot.odo.update();
        // Pinpoint heading is clockwise positive — flip it
        double currentHeading = -robot.odo.getPosition().getHeading(AngleUnit.DEGREES);
        currentHeading = (currentHeading % 360 + 360) % 360;

        // Compute shortest signed error (-180, 180]
        double error = ((targetAngleDeg - currentHeading + 540) % 360) - 180;

        double kP = 0.01;
        double turnPower = kP * error;
        turnPower = Range.clip(turnPower, -0.2, 0.2);

        if (Math.abs(error) <= 2) {
            stopMotors();
            return;
        }

        double minPower = 0.15;
        if (Math.abs(turnPower) < minPower)
            turnPower = Math.copySign(minPower, turnPower);

        robotCentricDrive(0, 0, -turnPower);
    }
    public void turnInDirection(double direction) {
        robotCentricDrive(0, 0, direction*0.1);
    }






    // --- Motor Power Handling ---
    private void applyMotorPowers() {
        leftFrontPower  *= speed;
        leftBackPower   *= speed;
        rightFrontPower *= speed;
        rightBackPower  *= speed;
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFrontPower  /= max;
        rightFrontPower /= max;
        leftBackPower   /= max;
        rightBackPower  /= max;

        robot.lf.setPower(leftFrontPower);
        robot.rf.setPower(rightFrontPower);
        robot.lb.setPower(leftBackPower);
        robot.rb.setPower(rightBackPower);
    }

    public void stopMotors() {
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }

    public TelemetryManager updateTelemetry(TelemetryManager telemetryM) {
        // Panels telemetry
        if (detailedPanels) {
            telemetryM.debug("Joystick Active", joystickActive);
            telemetryM.debug("Strafe", strafe);
            telemetryM.debug("Forward", forward);
            telemetryM.debug("Rotate", rotate);
            telemetryM.debug("Speed Multiplier", speed);
            telemetryM.debug("Deadzone", DEADZONE);

            telemetryM.debug("LB Power", leftBackPower);
            telemetryM.debug("LF Power", leftFrontPower);
            telemetryM.debug("RF Power", rightFrontPower);
            telemetryM.debug("RB Power", rightBackPower);
        }



        return telemetryM;
    }

}
