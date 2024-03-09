package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;

@TeleOp
@Config
public class LiftTest extends LinearOpMode {
    public static double LIFT_POWER_UP = 1;
    public static double LIFT_POWER_DOWN = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);
        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        Differential diffy = new Differential(this.hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();
            if (gamepad1.dpad_up) {
                diffy.tick(0, LIFT_POWER_UP, 0);
            } else if (gamepad1.dpad_down) {
                diffy.tick(0, -LIFT_POWER_DOWN, 0);
            }else {
                diffy.tick(0, 0, 0);
            }
            diffy.update();
            log.tick();
        }
    }
}
