package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Fold {
    Servo fold;

    public static double FOLD_INIT = 0;
    public static double FOLD_DRIVE = 0.74;

    public Fold(HardwareMap hardwareMap) {
        fold = hardwareMap.servo.get("fold");
    }

    public void setInit() {
        fold.setPosition(FOLD_INIT);
    }

    public void setDrive() {
        fold.setPosition(FOLD_DRIVE);
    }

    public void setPosition(double position) {
        fold.setPosition(position);
    }
}
