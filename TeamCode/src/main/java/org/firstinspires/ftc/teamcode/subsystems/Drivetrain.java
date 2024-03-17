package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.Log;


public class Drivetrain {
    public static double LATERAL_MULTIPLIER = 1.5;
    public DcMotor fl, fr, bl, br;

    public Drivetrain(HardwareMap hardwareMap) {

        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        MotorConfigurationType motorConfigurationType = fr.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        fr.setMotorType(motorConfigurationType);

        motorConfigurationType = fl.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        fl.setMotorType(motorConfigurationType);

        motorConfigurationType = br.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        br.setMotorType(motorConfigurationType);

        motorConfigurationType = bl.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        bl.setMotorType(motorConfigurationType);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }


    public void driveMecanum(double inAxial, double inLateral, double inYaw, double maxOutput) {

        double modMaintainMotorRatio;

        double inputAxial = (inAxial * maxOutput);
        double inputLateral = (inLateral * maxOutput) * LATERAL_MULTIPLIER; // TODO: tune
        double inputYaw = (inYaw * maxOutput);

        modMaintainMotorRatio = Math.max(Math.abs(inputAxial) + Math.abs(inputLateral) + Math.abs(inputYaw), 1);

        double leftFrontPower = (inputAxial + inputLateral + inputYaw) / modMaintainMotorRatio;
        double rightFrontPower = (inputAxial - inputLateral - inputYaw) / modMaintainMotorRatio;
        double leftBackPower = (inputAxial - inputLateral + inputYaw) / modMaintainMotorRatio;
        double rightBackPower = (inputAxial + inputLateral - inputYaw) / modMaintainMotorRatio;

        Log.getInstance()
                .add("leftFrontPower", String.valueOf(leftFrontPower))
                .add("rightFrontPower", String.valueOf(rightFrontPower))
                .add("leftBackPower", String.valueOf(leftBackPower))
                .add("rightBackPower", String.valueOf(rightBackPower));

        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void driveMecanumFieldCentric(double inAxial, double inLateral, double inYaw, double heading, double maxOutput) {

        double modMaintainMotorRatio;

        double inputAxial = (inAxial * maxOutput);
        double inputLateral = (inLateral * maxOutput) * LATERAL_MULTIPLIER;
        double inputYaw = (inYaw * maxOutput);

        double botHeading = heading;

        double adjLateral = inputLateral * Math.cos(botHeading) - inputAxial * Math.sin(botHeading);
        double adjAxial = inputLateral * Math.sin(botHeading) + inputAxial * Math.cos(botHeading);

        modMaintainMotorRatio = Math.max(Math.abs(inputAxial) + Math.abs(inputLateral) + Math.abs(inputYaw), 1);

        double leftFrontPower = (adjAxial + adjLateral + inputYaw) / modMaintainMotorRatio;
        double rightFrontPower = (adjAxial - adjLateral - inputYaw) / modMaintainMotorRatio;
        double leftBackPower = (adjAxial - adjLateral + inputYaw) / modMaintainMotorRatio;
        double rightBackPower = (adjAxial + adjLateral - inputYaw) / modMaintainMotorRatio;

        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setDriveMotorPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        fl.setPower(leftFrontPower);
        fr.setPower(rightFrontPower);
        bl.setPower(leftBackPower);
        br.setPower(rightBackPower);
    }
}