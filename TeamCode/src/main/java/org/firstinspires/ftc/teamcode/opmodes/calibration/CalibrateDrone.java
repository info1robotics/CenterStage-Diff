package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drone;

@Config
@Autonomous(group = "Calibration")
public class CalibrateDrone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drone drone = new Drone(this.hardwareMap);
        drone.setPosition(1);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad2.dpad_up)
                    drone.setPosition(1);
            if(gamepad2.dpad_down)
                    drone.setPosition(0);
        }
    }
}
