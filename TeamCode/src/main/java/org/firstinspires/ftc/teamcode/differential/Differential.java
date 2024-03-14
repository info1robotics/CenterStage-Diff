package org.firstinspires.ftc.teamcode.differential;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;

@Config
public class Differential {
    public static double M1_MULTIPLIER = 1.0;
    public static double M2_MULTIPLIER = 1.0;
    public static double M3_MULTIPLIER = 1.0;
    public static double[] LIFT_BOUND = new double[]{-50000, 0};
    public static double[] EXTENDO_BOUND = new double[]{0, 75000};
    public static double[] HANG_BOUND = new double[]{-1, 1};
    static double LIFT_CORRECTION = 4 / 8000d;
    static double EXTENDO_CORRECTION = 2 / 8000d;
    static double HANG_CORRECTION = 4 / 8000d;
    static double LIFT_DECEL_POS = -20000;
    static double EXTENDO_DECEL_POS = 10000;
    public Module<Double> prevPowers;

    public Module<Integer> targetTicks, prevTicks;
    public Module<Double> prevRealPowers; // powers including correction
    DcMotor parallel1, parallel2, perpendicular;
    double[] powers = new double[]{0, 0, 0};
    Module<Boolean> released;
    DifferentialTickBuffer differentialTickBuffer;

    public Differential(HardwareMap hardwareMap) {
        parallel1 = hardwareMap.get(DcMotor.class, "m1"); // Left, looking from intake
        parallel2 = hardwareMap.get(DcMotor.class, "m2"); // Right, looking from intake
        perpendicular = hardwareMap.get(DcMotor.class, "m3");

        differentialTickBuffer = new DifferentialTickBuffer();

        prevTicks = new Module<>(0, 0, 0);
        targetTicks = new Module<>(0, 0, 0);
        prevPowers = new Module<>(0d, 0d, 0d);
        prevRealPowers = new Module<>(0d, 0d, 0d);
        released = new Module<>(true, true, true);

        DcMotor[] motors = new DcMotor[]{parallel1, parallel2, perpendicular};

        perpendicular.setDirection(DcMotorSimple.Direction.REVERSE);

        for (int i = 0; i < 2; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfigurationType configurationType = motors[i].getMotorType();
            configurationType.setAchieveableMaxRPMFraction(1.0);
            motors[i].setMotorType(configurationType);
        }
    }

    public void update() {
        double max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
        max = Math.max(max, Math.abs(powers[2]));
        if (max > 1) {
            powers[0] /= max;
            powers[1] /= max;
            powers[2] /= max;
        }
        parallel1.setPower(powers[0] * M1_MULTIPLIER);
        parallel2.setPower(powers[1] * M2_MULTIPLIER);
        perpendicular.setPower(powers[2] * M3_MULTIPLIER);
    }

    public void reset() {
        powers = new double[]{0, 0, 0};
    }

    public boolean isClose(double current, double target, double zone) {
        return Math.abs(current - target) < zone;
    }

    public void outputHang(double power) {
        powers[0] += power;
        powers[1] -= power;
        prevRealPowers.setHang(power);
    }

    public void outputLift(double power) {
        double originalPower = power;
        power = -power;
        if (power > 0.7) power = 0.7;

        if (BulkReader.getInstance().getLiftTicks() > LIFT_DECEL_POS && originalPower < 0) {
            power = Math.abs(BulkReader.getInstance().getLiftTicks() / LIFT_DECEL_POS) * 0.6 + 0.2;
        }

        prevRealPowers.setLift(power);
        powers[0] += power;
        powers[1] += power;
        powers[2] += power;
    }

    public void outputExtendo(double power) {
        double currentPos = BulkReader.getInstance().getExtendoTicks();
        if (currentPos < EXTENDO_DECEL_POS && power < 0) {
            power = -(Math.abs(BulkReader.getInstance().getExtendoTicks() / EXTENDO_DECEL_POS) * 0.9 + 0.13);
        }

        prevRealPowers.setExtendo(power);

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
        final double originalHangPower = hangPower;
        final double originalLiftPower = liftPower;
        final double originalExtendoPower = extendoPower;

        differentialTickBuffer.addTicks(
                BulkReader.getInstance().getExtendoTicks(),
                BulkReader.getInstance().getLiftTicks(),
                BulkReader.getInstance().getHangTicks()
        );


        // Button released
        if (originalLiftPower == 0 && prevPowers.getLift() != 0) {
            released.setLift(true);
        }

        if (originalExtendoPower == 0 && prevPowers.getExtendo() != 0) {
            released.setExtendo(true);
        }

//        if (originalHangPower == 0 && prevPowers.getHang() != 0) {
//            released.setHang(true);
//        }

        // Reached stability after button release
        if (differentialTickBuffer.isExtendoStable() && released.getExtendo()) {
            targetTicks.setExtendo(BulkReader.getInstance().getExtendoTicks());
            released.setExtendo(false);
        }

        if (differentialTickBuffer.isLiftStable() && released.getLift()) {
            targetTicks.setLift(BulkReader.getInstance().getLiftTicks());
            released.setLift(false);
        }

//        if (differentialTickBuffer.isHangStable() && released.getHang()) {
//            targetTicks.setHang(BulkReader.getInstance().getHangTicks());
//            released.setHang(false);
//        }

        // Calculate correction power and apply if reached stability
        if (extendoPower == 0 && !released.getExtendo()) {
            extendoPower = -(BulkReader.getInstance().getExtendoTicks() - targetTicks.getExtendo()) * EXTENDO_CORRECTION;
        }

        if (liftPower == 0 && !released.getLift()) {
            liftPower = (BulkReader.getInstance().getLiftTicks() - targetTicks.getLift()) * LIFT_CORRECTION;
        }

//        if (hangPower == 0 && !released.getHang()) {
//            hangPower = (BulkReader.getInstance().getHangTicks() - targetTicks.getHang()) * HANG_CORRECTION;
//        }

        // Bounds
        if (BulkReader.getInstance().getLiftTicks() < LIFT_BOUND[0] && liftPower > 0) {
            liftPower = 0;
        } else if (BulkReader.getInstance().getLiftTicks() > LIFT_BOUND[1] && liftPower < 0) {
            liftPower = 0;
        }

        if (BulkReader.getInstance().getExtendoTicks() < EXTENDO_BOUND[0] && extendoPower < 0) {
            extendoPower = 0;
        } else if (BulkReader.getInstance().getExtendoTicks() > EXTENDO_BOUND[1] && extendoPower > 0) {
            extendoPower = 0;
        }


        outputHang(hangPower);
        outputLift(liftPower);
        outputExtendo(extendoPower);

        prevTicks.setExtendo(BulkReader.getInstance().getExtendoTicks());
        prevTicks.setLift(BulkReader.getInstance().getLiftTicks());
        prevTicks.setHang(BulkReader.getInstance().getHangTicks());

        prevPowers.setHang(originalHangPower);
        prevPowers.setLift(originalLiftPower);
        prevPowers.setExtendo(originalExtendoPower);

        Log.getInstance()
                .add("Hang", hangPower)
                .add("Extendo", extendoPower)
                .add("Extendo Ticks", BulkReader.getInstance().getExtendoTicks())
                .add("Extendo Target", targetTicks.getExtendo())
                .add("Power 0", powers[0])
                .add("Power 1", powers[1])
                .add("Power 2", powers[2])
                .add("Lift", liftPower)
                .add("Lift Ticks", BulkReader.getInstance().getLiftTicks())
                .add("Lift Target", targetTicks.getLift());

    }
}
