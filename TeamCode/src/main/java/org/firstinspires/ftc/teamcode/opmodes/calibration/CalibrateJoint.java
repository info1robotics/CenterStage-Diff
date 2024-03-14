package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Joint;

@Config
@Autonomous(group = "Calibration")
public class CalibrateJoint extends LinearOpMode {
    public static double position = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        Joint joint = new Joint(this.hardwareMap);

        while (opModeInInit()) {
            log.add("Align the claw with the intake's slope.");
            log.tick();
        }

        while (opModeIsActive() && !isStopRequested()) {
            joint.setPosition(position);
            log.tick();
        }
    }
}
