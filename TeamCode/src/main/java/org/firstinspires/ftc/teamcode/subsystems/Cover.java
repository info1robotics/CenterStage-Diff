package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Cover {
    ServoImplEx cover;

    public static double COVER_CLOSED = 0.9;
    public static double COVER_OPEN = 0.1;

    public Cover(HardwareMap hardwareMap) {
        cover = hardwareMap.get(ServoImplEx.class, "cover");
        cover.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setClosed() {
        cover.setPosition(COVER_CLOSED);
    }

    public void setOpen() {
        cover.setPosition(COVER_OPEN);
    }

    public void setPosition(double position) {
        cover.setPosition(position);
    }

    public boolean isClosed() {
        return cover.getPosition() == COVER_CLOSED;
    }

    public boolean isOpen() {
        return cover.getPosition() == COVER_OPEN;
    }

    public void toggle() {
        if (isClosed()) setOpen();
        else setClosed();
    }
}
