package org.firstinspires.ftc.teamcode.enums;

public enum AutoStartPos {
    UNKNOWN,
    TELE,
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT;


    public boolean isRed() {
        return this == RED_LEFT || this == RED_RIGHT;
    }

    public boolean isBlue() {
        return this == BLUE_LEFT || this == BLUE_RIGHT;
    }
}