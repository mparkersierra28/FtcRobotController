package org.firstinspires.ftc.teamcode.tests.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.software.CameraQR;
import org.firstinspires.ftc.teamcode.software.MecanumDrive;
import org.firstinspires.ftc.teamcode.software.Physics;

@Configurable
@TeleOp(name="Gate PushGreg", group = "1")
public class GregGatePush extends OpMode {
    private RobotHardware robot;
    private MecanumDrive drive;
    private CameraQR camera;
    private Physics physics;

    public TelemetryManager telemetryM;

    // Launching Degree Position

    public static double launchPower = 1.0;
    public static double servoPower = 0.5;
    public static double gateSPower = 0.2;

    private boolean fieldCentric = true;
    private boolean twoPlayers = false;
    private boolean aMenuPressed1;
    private boolean bMenuPressed1;
    private boolean aMenuPressed2;

    // Mode states gamepad1
    private boolean feederRunning1 = false;
    private boolean launcherRunning1 = false;
    private boolean humanRunning1 = false;
    private boolean launching = false;
    // Buttons 1
    private boolean prevA1 = false;
    private boolean prevB1 = false;
    private boolean prevX1 = false;
    private boolean prevY1 = false;

    // Buttons 2
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevLB2 = false;

    // Mode states gamepad2
    private boolean launchMode = false;
    private boolean middleMode = false;
    private boolean intakeMode = false;
    private boolean allServosMode = false;
    private boolean allServosAndLaunchMode = false;

    private boolean somethingRunning = false;

    public static int lErrorMargin = 40;

    private int goalXPos = 140, goalYPos = 4;

    private double humanStartTime;
    public static int launchBigVel = 920;
    public static int launchSmallVel = 750;
    public static int launchReallyClose = 680;

    private double adjustedServoPower;

    private double gatePushStartTime;

    private boolean isAtGatePos = true;
    public static double sGatePos = 0.7;
    public static double sPushPos = 0;
    public static double gateDelay = 200;
    public static double pushDelay = 1500;




    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware(hardwareMap);
        robot.setRobotConfig();

        // Get the driving class
        drive = new MecanumDrive(robot);

        camera = new CameraQR(robot);

        physics = new Physics(robot);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized - Panels Connected");
    }
    @Override
    public void init_loop() {
        // Toggle field-centric mode
        if (gamepad1.a && !aMenuPressed1) {
            fieldCentric = !fieldCentric;
            aMenuPressed1 = true;
        } else if (!gamepad1.a) {
            aMenuPressed1 = false;
        }

        // Toggle alliance (red/blue)
        if (gamepad1.b && !bMenuPressed1) {
            robot.alliance = (robot.isRedAlliance())
                    ? RobotHardware.Alliance.BLUE
                    : RobotHardware.Alliance.RED;
            bMenuPressed1 = true;
        } else if (!gamepad1.b) {
            bMenuPressed1 = false;
        }

        // Toggle two player mode
//        if (gamepad2.a && !aMenuPressed2) {
//            twoPlayers = !twoPlayers;
//            aMenuPressed2 = true;
//        } else if (!gamepad2.a) {
//            aMenuPressed2 = false;
//        }

        telemetry.addData("Menu: Field Centric Mode", fieldCentric ? "ON" : "OFF");
        telemetry.addLine("Press A to toggle");

        telemetry.addData("Menu: Alliance", robot.isRedAlliance() ? "RED" : "BLUE");
        telemetry.addLine("Press B to toggle");

//        telemetry.addData("Menu: Two Player Mode", twoPlayers ? "YES" : "NO");
//        telemetry.addLine("Gamepad2: Press A to toggle");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setFieldCentric(fieldCentric);

        if (!robot.isRedAlliance()) goalYPos = goalXPos;
        robot.gatePush.setPosition(sGatePos);
    }
    @Override
    public void loop() {

        somethingRunning = false;
        gamePad1Controls();

        //if (twoPlayers) gamepad2Controls();

        telemetryM.update(telemetry);
    }
    private void gamePad1Controls() {
        double triggerValue = gamepad1.right_trigger;
        double speed = 1.0 - (0.9 * triggerValue);
        adjustedServoPower = servoPower * speed;

        // Handle driving
        // --- Rotate to clip angle ---
        if (gamepad1.dpad_left) {
            //drive.turnToAngle(gamepad1.dpad_down ? clipAngleClose : clipAngleFar);
            drive.turnInDirection(physics.getHeadingError(goalXPos, goalYPos));
//            telemetryM.debug(Math.toDegrees(physics.getHeadingError(144, 0)));
//            telemetryM.debug(Math.toDegrees(physics.getAngleToPoint(144, 0)));
//            telemetryM.debug("Current Angle: ", robot.odo.getHeading(AngleUnit.DEGREES));
//            telemetryM.debug(robot.odo.getPosX(DistanceUnit.INCH), robot.odo.getPosY(DistanceUnit.INCH));
        } else {
            drive.update(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.right_stick_y
            );
        }
        // --- Emergency Stop ---
        if (gamepad1.left_bumper) {
            launcherRunning1 = false;
            launching = false;
            feederRunning1 = false;
            humanRunning1 = false;
            stopAllLaunching();
            return;
        }

        // --- Reset Odo Dir ---
        if (gamepad1.right_bumper) {
            robot.odo.setPosition(new Pose2D(DistanceUnit.INCH, 8.5, 72, AngleUnit.RADIANS, 0));
        }

        // --- A Toggle (Launcher) ---
        if (gamepad1.a && !prevA1) {
            // If feeder is running, stop it (simulate sleep)
            if (feederRunning1) {
                feederRunning1 = false;
                stopIntake();
            }

            launcherRunning1 = !launcherRunning1;
            if (launcherRunning1) {
//                targetVelocity = physics.getVelocityTpS(goalXPos, goalYPos);
//                // velocity (m/s)/circumference(0.096 * PI)
//                targetVelocity = targetVelocity/(0.096 * Math.PI);
            } else {
                stopAllLaunching();
            }
        }
        prevA1 = gamepad1.a;


        // --- B Toggle (Feeder) ---
        if (gamepad1.b && !prevB1) {
            feederRunning1 = !feederRunning1;
            if (!feederRunning1) {
                stopIntake();
            }
        }
        prevB1 = gamepad1.b;

        // --- X Toggle (Human Mode) ---
        if (gamepad1.x && !prevX1) {
            // Toggle human mode
            humanRunning1 = !humanRunning1;

            if (humanRunning1) {
                humanStartTime = System.currentTimeMillis();
                // Turn off launcher + feeder for safety
                launcherRunning1 = false;
                feederRunning1 = false;
                stopAllLaunching();
            } else {
                // Human turned off: stop all motors
                stopAllLaunching();
            }
        }
        prevX1 = gamepad1.x;
        double targetVelocity = gamepad1.dpad_down ? launchBigVel : (gamepad1.dpad_up ? launchReallyClose : launchSmallVel);

        // --- Launcher Running ---
        if (launcherRunning1) {
            somethingRunning = true;
            if (Math.abs(targetVelocity-robot.launcherR.getVelocity())<=lErrorMargin&&Math.abs(targetVelocity-robot.launcherL.getVelocity())<=lErrorMargin){
                gamepad1.rumble(0.6, 0.6, 40);
            }
            if (gamepad1.y && !prevY1) {
                launching = !launching;
                if (launching) {
                    //gatePushStartTime = System.currentTimeMillis();
                    feederRunning1 = false;
                }
                if (!launching) stopIntake();
            }
            prevY1 = gamepad1.y;
            if (launching) {
                launchBalls();
            }


            robot.launcherR.setVelocity(targetVelocity);
            robot.launcherL.setVelocity(targetVelocity);
        }

        // --- Feeder Running ---
        if (feederRunning1) {
            somethingRunning = true;

            runIntake();
        }

        // --- Human Running (Toggle Mode) ---
        if (humanRunning1) {
            somethingRunning = true;

//            if (humanStartTime < System.currentTimeMillis() - 500) robot.thirdUpS.setPower(0.1);
//            else robot.thirdUpS.setPower(-0.1);

            robot.launcherR.setPower(-launchPower/5);
            robot.launcherL.setPower(-launchPower/5);
            robot.secondUpS.setPower(-adjustedServoPower / 4);
            robot.firstUpS.setPower(-adjustedServoPower / 5);
            robot.intakeS.setPower(0.01);
        }
        telemetryM.addData("Velocity", robot.launcherR.getVelocity());
        telemetryM.addData("Error", robot.launcherR.getVelocity()-targetVelocity);
    }





    private void stopIntake() {
        robot.secondUpS.setPower(0.0);
        robot.firstUpS.setPower(0.0);
        robot.intakeS.setPower(0.0);
    }
    private void stopAllLaunching() {
        robot.launcherR.setPower(0.0);
        robot.launcherL.setPower(0.0);
        //robot.thirdUpS.setPower(0.0);
        robot.secondUpS.setPower(0.0);
        robot.firstUpS.setPower(0.0);
        robot.intakeS.setPower(0.0);
        robot.gatePush.setPosition(sGatePos);
    }
    private void runIntake() {
        robot.secondUpS.setPower(adjustedServoPower);
        robot.firstUpS.setPower(adjustedServoPower);
        robot.intakeS.setPower(adjustedServoPower);
    }
    private void launchBalls() {
        if (isAtGatePos && System.currentTimeMillis() - pushDelay > gatePushStartTime) {
            robot.gatePush.setPosition(sPushPos);
            stopIntake();
            isAtGatePos = false;
            gatePushStartTime = System.currentTimeMillis();
        } else if (!isAtGatePos && System.currentTimeMillis() - gateDelay > gatePushStartTime) {
            robot.gatePush.setPosition(sGatePos);
            isAtGatePos = true;
            runIntake();
            gatePushStartTime = System.currentTimeMillis();
        }

    }
}



