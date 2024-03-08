package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo clawLeft, clawRight;

    static double CLAW_LEFT_OPEN = 1;
    static double CLAW_LEFT_CLOSE = 0.54;
    static double CLAW_RIGHT_OPEN = 0;
    static double CLAW_RIGHT_CLOSE = 0.34;

    public Claw(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.servo.get("clawLeft"); // When looking from the intake
        clawRight = hardwareMap.servo.get("clawRight");
    }

    public enum Side {
        LEFT, RIGHT
    }

    public void close() {
        clawLeft.setPosition(CLAW_LEFT_CLOSE);
        clawRight.setPosition(CLAW_RIGHT_CLOSE);
    }

    public void open() {
        clawLeft.setPosition(CLAW_LEFT_OPEN);
        clawRight.setPosition(CLAW_RIGHT_OPEN);
    }


    public void close(Side side) {
        if (side == Side.LEFT) {
            clawLeft.setPosition(CLAW_LEFT_CLOSE);
        } else {
            clawRight.setPosition(CLAW_RIGHT_CLOSE);
        }
    }

    public void open(Side side) {
        if (side == Side.LEFT) {
            clawLeft.setPosition(CLAW_LEFT_OPEN);
        } else {
            clawRight.setPosition(CLAW_RIGHT_OPEN);
        }
    }

    public void toggle() {
        if (clawLeft.getPosition() == CLAW_LEFT_OPEN) {
            close();
        } else {
            open();
        }
    }

    public void toggle(Side side) {
        if (side == Side.LEFT) {
            if (clawLeft.getPosition() == CLAW_LEFT_OPEN) {
                close(side);
            } else {
                open(side);
            }
        } else {
            if (clawRight.getPosition() == CLAW_RIGHT_OPEN) {
                close(side);
            } else {
                open(side);
            }
        }
    }
}
