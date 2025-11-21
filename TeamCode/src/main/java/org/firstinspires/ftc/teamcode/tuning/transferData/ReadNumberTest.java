package org.firstinspires.ftc.teamcode.tuning.transferData;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.content.SharedPreferences;
import android.content.Context;

@Disabled
@TeleOp(name="Read Number Test")
public class ReadNumberTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Press START to read saved value.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.start) {
                int value = readNumber();
                telemetry.addData("Saved Value", value);
                telemetry.update();
                sleep(500);
            }
        }
    }

    private int readNumber() {
        SharedPreferences preferences =
                hardwareMap.appContext.getSharedPreferences("RobotData", Context.MODE_PRIVATE);
        return preferences.getInt("autoVal", -1); // default if nothing saved
    }
}
