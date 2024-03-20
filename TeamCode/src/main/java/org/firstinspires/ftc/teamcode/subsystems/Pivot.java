package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Pivot {
    ServoImplEx pivotLeft, pivotRight;

    public static double PIVOT_COLLECT = 0.09;
    public static double PIVOT_SCORE = 1;
    PivotState pivotState = PivotState.NONE;

    public enum PivotState {
        COLLECT,
        SCORE,
        NONE
    }

    public Pivot(HardwareMap hardwareMap) {
        pivotLeft = hardwareMap.get(ServoImplEx.class, "pivotLeft");
        pivotRight = hardwareMap.get(ServoImplEx.class, "pivotRight");


        pivotLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pivotRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pivotRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setCollect() {
        pivotLeft.setPosition(PIVOT_COLLECT);
        pivotRight.setPosition(PIVOT_COLLECT);
        pivotState = PivotState.COLLECT;
    }

    public void setScore() {
        pivotLeft.setPosition(PIVOT_SCORE);
        pivotRight.setPosition(PIVOT_SCORE);
        pivotState = PivotState.SCORE;
    }

    public void setPosition(double position) {
        pivotLeft.setPosition(position);
        pivotRight.setPosition(position);
        pivotState = PivotState.NONE;
    }

    public boolean is(PivotState pivotState) {
        return this.pivotState == pivotState;
    }
}
