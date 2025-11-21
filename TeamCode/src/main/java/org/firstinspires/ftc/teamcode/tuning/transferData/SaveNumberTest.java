package org.firstinspires.ftc.teamcode.tuning.transferData;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.content.SharedPreferences;
import android.content.Context;

@Disabled
@TeleOp(name="Save Number Test")
public class SaveNumberTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Press A/B/X/Y to pick a number.\nPress START to save.");
        telemetry.update();

        int pickedNumber = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) pickedNumber = 1;
            if (gamepad1.b) pickedNumber = 2;
            if (gamepad1.x) pickedNumber = 3;
            if (gamepad1.y) pickedNumber = 4;

            telemetry.addData("Selected", pickedNumber);
            telemetry.addLine("Press START to save.");
            telemetry.update();

            // Save when you press START
            if (gamepad1.start) {
                saveNumber(pickedNumber);
                telemetry.addLine("Saved!");
                telemetry.update();
                sleep(500);
            }
        }
    }

    private void saveNumber(int number) {
        SharedPreferences preferences =
                hardwareMap.appContext.getSharedPreferences("RobotData", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = preferences.edit();
        editor.putInt("autoVal", number);
        editor.apply();
    }
}
