package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Cover {
    ServoImplEx cover;

    public static double COVER_CLOSED = 0.8;
    public static double COVER_OPEN = 0.0;
    boolean open = false;

    public Cover(HardwareMap hardwareMap) {
        cover = hardwareMap.get(ServoImplEx.class, "cover");
        cover.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void close() {
        open = false;
        cover.setPosition(COVER_CLOSED);
    }

    public void open() {
        open = true;
        cover.setPosition(COVER_OPEN);
    }

    public void setPosition(double position) {
        cover.setPosition(position);
    }

    public boolean isOpen() {
        return open;
    }

    public boolean isClosed() {
        return !open;
    }

    public void toggle() {
        if (isClosed()) open();
        else close();
    }
}
