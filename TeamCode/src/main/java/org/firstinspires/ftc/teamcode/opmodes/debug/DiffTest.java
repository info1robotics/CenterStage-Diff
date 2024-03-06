package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Differential;

@TeleOp
public class DiffTest extends LinearOpMode {

    LynxModule controlHub, expansionHub;
    LynxModule.BulkData controlHubData, expansionHubData;

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);
        BulkReader bulkReader = new BulkReader(this.hardwareMap);

        Differential diffy = new Differential(this.hardwareMap);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();
            if (gamepad1.a) {
                diffy.tick(1, 0, 0);
            } else if (gamepad1.b) {
                diffy.tick(0, 0, -.4);
            } else if (gamepad1.y) {
                diffy.tick(0, 0, .4);
            } else {
                diffy.tick(0, 0, 0);
            }
            Log.getInstance()
               .add("Encoder", bulkReader.controlHubData.getMotorCurrentPosition(0));
            diffy.update();
            log.tick();
        }
    }
}
