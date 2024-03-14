package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
@Autonomous(group = "Calibration")
public class CalibratePivot extends LinearOpMode {
    public static double position = 1.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        Pivot pivot = new Pivot(this.hardwareMap);

        while (opModeInInit()) {
            log.add("Align the pivot parallel to the ground.");
            log.tick();
        }

        while (opModeIsActive() && !isStopRequested()) {
            pivot.setPosition(position);
            log.tick();
        }
    }
}
