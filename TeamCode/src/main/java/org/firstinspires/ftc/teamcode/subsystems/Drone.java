package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    Servo drone;

    public Drone(HardwareMap hardwareMap) {
        this.drone = hardwareMap.servo.get("drone");
    }

    public void setPosition(double pos) {
        drone.setPosition(pos);
    }

    public double getPosition() {
        return drone.getPosition();
    }
}
