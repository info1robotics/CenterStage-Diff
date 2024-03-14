package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Cover;

@Config
@Autonomous(group = "Calibration")
public class CalibrateCover extends LinearOpMode {
    public static double position = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        Cover cover = new Cover(this.hardwareMap);

        while (opModeInInit()) {
            log.add("Close the cover all the way.");
            log.tick();
        }

        while (opModeIsActive() && !isStopRequested()) {
            cover.setPosition(position);
            log.tick();
        }
    }
}
