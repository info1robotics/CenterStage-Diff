package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;

public class Differential {
    DcMotor m1, m2, m3;
    double[] powers = new double[]{0, 0, 0};


    // hang, lift, extendo
    double[] targets = new double[]{0, 0, 0};
    double[] outputPowers = new double[]{0, 0, 0};

    public Differential(HardwareMap hardwareMap) {
        m1 = hardwareMap.get(DcMotor.class, "diffPara1");
        m2 = hardwareMap.get(DcMotor.class, "diffPara2");
        m3 = hardwareMap.get(DcMotor.class, "diffPerp");

        m3.setDirection(DcMotorSimple.Direction.REVERSE);

//        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        double max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
        max = Math.max(max, Math.abs(powers[2]));
        if (max > 1) {
            powers[0] /= max;
            powers[1] /= max;
            powers[2] /= max;
        }
        m1.setPower(powers[0]);
        m2.setPower(powers[1]);
        m3.setPower(powers[2]);
    }

    public void reset() {
        outputPowers = new double[]{0, 0, 0};
        powers = new double[]{0, 0, 0};
    }

    public void outputHang(double power) {
        outputPowers[0] = power;
        powers[0] += power;
        powers[1] -= power;

    }

    public void outputLift(double power) {
        outputPowers[1] = power;
        powers[0] += power;
        powers[1] += power;
        powers[2] += power;
    }

    public void outputExtendo(double power) {
        outputPowers[2] = power;
        powers[0] += power;
        powers[1] += power;
        powers[2] -= power;
    }

//    public void moveDiff(double hangPower, double liftPower, double extendoPower) {
//        reset();
//        outputHang(hangPower);
//        outputLift(liftPower);
//        outputExtendo(extendoPower);
//        update();
//    }

    public void tick(double hangPower, double liftPower, double extendoPower) {
//        if (extendoPower != 0) targets[2] = BulkReader.getInstance().getExtendoTicks();
//        else if (isNotClose(BulkReader.getInstance().getExtendoTicks(), targets[2])) {
////            double power = (BulkReader.getInstance().getExtendoTicks() > targets[2]) ? -0.004 : 0.004;
////            extendoPower += power;
//        }

        outputHang(hangPower);
        outputLift(liftPower);
        outputExtendo(extendoPower);

        Log.getInstance()
                .add("Hang", hangPower)
                .add("Lift", liftPower)
                .add("Extendo", extendoPower)
                .add("Extendo Ticks", BulkReader.getInstance().getExtendoTicks())
                .add("Extendo Target", targets[2])
                .add("Power 0", powers[0])
                .add("Power 1", powers[1])
                .add("Power 2", powers[2])
                .add("isNotClose", isNotClose(BulkReader.getInstance().getExtendoTicks(), targets[2]));
    }

    public boolean isNotClose(double target, double current) {
        return !(Math.abs(target - current) < 10000);
    }
}
