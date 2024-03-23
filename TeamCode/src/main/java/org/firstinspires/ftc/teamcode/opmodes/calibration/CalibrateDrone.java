package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drone;

@Config
@Autonomous(group = "Calibration")
public class CalibrateDrone extends LinearOpMode {
    public static double pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Drone drone = new Drone(this.hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drone.setPosition(pos);
        }
    }
}
