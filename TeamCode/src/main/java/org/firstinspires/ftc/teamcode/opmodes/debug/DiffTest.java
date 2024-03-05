package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.opmodes.subsystems.Differential;

public class DiffTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new Log(telemetry);

        Differential diffy = new Differential(this.hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            diffy.reset();
            if (gamepad1.a) {
                diffy.outputDiff(1, 0 , 0);
            } else if (gamepad2.b) {
                diffy.outputDiff(0, 1, 0);
            } else if (gamepad1.y) {
                diffy.outputDiff(0, 0, 1);
            }
            diffy.update();
        }
    }
}
