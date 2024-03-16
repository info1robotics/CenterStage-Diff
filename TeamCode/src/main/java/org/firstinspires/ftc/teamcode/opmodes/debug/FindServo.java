package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "Debug")
public class FindServo extends LinearOpMode {
    public static String name = "asd";
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            servo.setPosition(1);
        }
    }
}
