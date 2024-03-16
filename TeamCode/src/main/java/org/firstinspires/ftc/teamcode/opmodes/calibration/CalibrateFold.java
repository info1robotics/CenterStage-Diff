package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Fold;

@Config
@Autonomous(group = "Calibration")
public class CalibrateFold extends LinearOpMode {
    public static double position = 0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        Fold fold = new Fold(this.hardwareMap);

        while (opModeInInit()) {
            log.add("Align the fold parallel to the ground.");
            log.tick();
        }

        while (opModeIsActive() && !isStopRequested()) {
            fold.setPosition(position);
            log.tick();
        }
    }
}