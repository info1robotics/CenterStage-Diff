package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Cover {
    Servo cover;

    public static double COVER_CLOSED = 0.9;
    public static double COVER_OPEN = 0;

    public Cover(HardwareMap hardwareMap) {
        cover = hardwareMap.servo.get("cover");
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
}
