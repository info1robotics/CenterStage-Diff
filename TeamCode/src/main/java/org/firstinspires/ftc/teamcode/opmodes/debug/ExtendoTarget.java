package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;

@Autonomous
public class ExtendoTarget extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        BulkReader reader = new BulkReader(hardwareMap);
        Differential diffy = new Differential(hardwareMap);
        waitForStart();
        diffy.targetTicks.setExtendo(40000);
        while (opModeIsActive()) {
            reader.read();
            diffy.reset();

            diffy.tick(0, 0, 0);
            diffy.update();
            log.tick();
        }
    }
}
