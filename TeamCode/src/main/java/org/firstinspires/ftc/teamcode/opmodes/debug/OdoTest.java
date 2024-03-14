package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.roadrunner.odometry.Odometry;

@Autonomous
public class OdoTest extends LinearOpMode {
    private final Odometry leftOdo = new Odometry(3, false);
    private final Odometry rightOdo = new Odometry(0, true);
    private final Odometry frontOdo = new Odometry(1, false);

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        BulkReader bulkReader = new BulkReader(this.hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            log.add("Odo Left", leftOdo.getCurrentPosition());
            log.add("Odo Right", rightOdo.getCurrentPosition());
            log.add("Odo Front", frontOdo.getCurrentPosition());
            log.tick();
        }
    }
}
