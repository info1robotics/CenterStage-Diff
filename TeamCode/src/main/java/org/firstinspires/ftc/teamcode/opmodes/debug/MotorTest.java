package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FL=hardwareMap.get(DcMotor.class,"BR");//change the name of the motor
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            FL.setPower(1);
        }
    }
}
