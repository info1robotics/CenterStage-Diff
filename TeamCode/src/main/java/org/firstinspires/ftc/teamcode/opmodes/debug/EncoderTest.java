package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Log;

import java.util.List;

@TeleOp(group = "Debug")
public class EncoderTest extends LinearOpMode {
    LynxModule controlHub, expansionHub;
    LynxModule.BulkData controlHubData, expansionHubData;

    @Override
    public void runOpMode() throws InterruptedException {

        Log log = new Log(telemetry);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        if (allHubs.get(0).isParent()) {
            controlHub = allHubs.get(0);
//            expansionHub = allHubs.get(1);
        } else {
            controlHub = allHubs.get(1);
//            expansionHub = allHubs.get(0);
        }

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controlHubData = controlHub.getBulkData();
            double ticks = controlHubData.getMotorCurrentPosition(0);
            log.add("Ticks", ticks);
            log.add("Time", System.currentTimeMillis());
            log.tick();
        }
    }
}
