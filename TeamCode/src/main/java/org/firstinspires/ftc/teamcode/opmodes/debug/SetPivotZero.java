package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "Debug")
public class SetPivotZero extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo pivotLeft = hardwareMap.servo.get("pivotLeft");
        Servo pivotRight = hardwareMap.servo.get("pivotRight");

        pivotRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pivotLeft.setPosition(0);
            pivotRight.setPosition(0);
        }
    }
}
