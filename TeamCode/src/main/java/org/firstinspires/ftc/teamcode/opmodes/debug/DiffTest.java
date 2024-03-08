package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;

@TeleOp
@Config
public class DiffTest extends LinearOpMode {

    public static double LIFT_POWER = 0.0;
    public static double EXTENDO_POWER = 0.0;
    public static double HANG_POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);
        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        bulkReader.read();
        Differential diffy = new Differential(this.hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();
            if (gamepad1.a) {
                diffy.tick(HANG_POWER, LIFT_POWER, EXTENDO_POWER);
            } else {
                diffy.tick(0, 0, 0);
            }
            diffy.update();
            log.tick();
        }
    }
}
