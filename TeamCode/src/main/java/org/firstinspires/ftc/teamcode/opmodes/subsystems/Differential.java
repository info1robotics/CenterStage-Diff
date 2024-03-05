package org.firstinspires.ftc.teamcode.opmodes.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Differential {
    DcMotor m1,m2,m3;
    double[] powers = new double[]{0,0,0};
    public Differential(HardwareMap hardwareMap) {
        m1 = hardwareMap.get(DcMotor.class, "diffPara1");
        m2 = hardwareMap.get(DcMotor.class, "diffPara2");
        m3 = hardwareMap.get(DcMotor.class, "diffPerp");

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void update(){

        //normalize all the powers
        double max = Math.max(Math.abs(powers[0]), Math.abs(powers[1]));
        max = Math.max(max, Math.abs(powers[2]));
        if(max>1){
            powers[0]/=max;
            powers[1]/=max;
            powers[2]/=max;
        }
        m1.setPower(powers[0]);
        m2.setPower(powers[1]);
        m3.setPower(powers[2]);
    }
    public void reset(){
        powers = new double[]{0,0,0};
    }

    public void outputHang(double power){
        powers[0]+=power;
        powers[1]-=power;

    }
    public void outputLift(double power){
        powers[0]+=power;
        powers[1]+=power;
        powers[2]+=power;
    }
    public void outputExtendo(double power){
        powers[0]+=power;
        powers[1]+=power;
        powers[2]-=power;
    }
    public void moveDiff(double hangPower, double liftPower, double extendoPower) {
        reset();
        outputHang(hangPower);
        outputLift(liftPower);
        outputExtendo(extendoPower);
        update();
    }

    public void outputDiff(double hangPower, double liftPower, double extendoPower) {
        outputHang(hangPower);
        outputLift(liftPower);
        outputExtendo(extendoPower);
    }
}
