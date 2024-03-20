package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Intake {
    DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        MotorConfigurationType motorConfigurationType = intake.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(motorConfigurationType);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void reverse() {
        intake.setPower(-1);
    }

    public void take() {
        intake.setPower(1);
    }
}
