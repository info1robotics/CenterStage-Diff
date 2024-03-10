package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@TeleOp(name = "Extendo Speed Test")
public class ExtendoSpeedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);

        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        Differential diffy = new Differential(this.hardwareMap);


        waitForStart();

        long start = System.currentTimeMillis();
        long finishTime = System.currentTimeMillis();

        double extendTime = -1;

        boolean finished = false;

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();

            if (!finished) {
                diffy.tick(0, 0, 1);
            } else {
                diffy.tick(0, 0, -1);
            }

            if (diffy.prevRealPowers.getExtendo() < 0.18 && !finished) {
                extendTime = (System.currentTimeMillis() - start) / 1000d;
                finished = true;
                finishTime = System.currentTimeMillis();
            }

            if (finished && (System.currentTimeMillis() - finishTime) > 4000) {
                stop();
            }
            Log.getInstance().add("Extendo Prev", diffy.prevRealPowers.getExtendo());
            Log.getInstance().add("Took " + extendTime + "s to extend.");

            diffy.update();
            log.tick();
        }
    }
}
