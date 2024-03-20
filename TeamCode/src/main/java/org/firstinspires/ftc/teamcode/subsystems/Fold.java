package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Fold {
    ServoImplEx fold;

    public static double FOLD_INIT = 0;
    public static double FOLD_UP = 0.25;
    public static double FOLD_DRIVE = 0.95;

    // auto positions top to bottom
    public static double[] autoPositions = {0.7, 0.75, 0.80, 0.9, FOLD_DRIVE, FOLD_DRIVE};

    public Fold(HardwareMap hardwareMap) {
        fold = hardwareMap.get(ServoImplEx.class, "fold");
        fold.setPwmRange(new PwmControl.PwmRange(500, 2500));
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

    public double getPosition() {
        return fold.getPosition();
    }
}
