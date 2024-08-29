package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Joint {
    ServoImplEx joint;

    public static double JOINT_COLLECT = 0.7; // TODO: find positions
    public static double JOINT_SCORE = 0.25;
    public static double JOINT_PHASE=0.68;
    public static double JOINT_TRANSITION = 1.0;

    public Joint(HardwareMap hardwareMap) {
        joint = hardwareMap.get(ServoImplEx.class, "joint");
    }

    public void setCollect() {
        joint.setPosition(JOINT_COLLECT);
    }

    public void setTransition() {
        joint.setPosition(JOINT_TRANSITION);
    }
    public void setPhase() {
        joint.setPosition(JOINT_PHASE);
    }

    public void setScore() {
        joint.setPosition(JOINT_SCORE);
    }

    public void setPosition(double position) {
        joint.setPosition(position);
    }
}
