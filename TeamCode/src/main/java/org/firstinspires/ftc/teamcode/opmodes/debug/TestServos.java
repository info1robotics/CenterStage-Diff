package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class TestServos extends LinearOpMode {
    public static double pos4 = 0;
    public static double pos5 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo4 = hardwareMap.servo.get("servo4");
        Servo servo5 = hardwareMap.servo.get("servo5");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            servo4.setPosition(pos4);
            servo5.setPosition(pos5);
        }
    }
}
