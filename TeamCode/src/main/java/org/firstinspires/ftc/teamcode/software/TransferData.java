package org.firstinspires.ftc.teamcode.software;


import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferData {

    public static void saveAlliance(HardwareMap hw, int alliance) {
        SharedPreferences preferences =
                hw.appContext.getSharedPreferences("RobotData", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = preferences.edit();
        editor.putInt("Alliance", alliance);
        editor.apply();
    }

    public static int getAlliance(HardwareMap hw) {
        SharedPreferences preferences =
                hw.appContext.getSharedPreferences("RobotData", Context.MODE_PRIVATE);
        return preferences.getInt("Alliance", 0); // default if nothing saved
    }
}
