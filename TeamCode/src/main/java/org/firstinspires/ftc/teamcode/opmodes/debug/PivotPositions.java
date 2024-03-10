package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
@Autonomous(group = "Debug")
public class PivotPositions extends LinearOpMode {
    public static double pos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pivot pivot = new Pivot(this.hardwareMap);
        Log log = new Log(this.telemetry);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pivot.setPosition(pos);
        }
    }
}
