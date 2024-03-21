package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReader {
    LynxModule controlHub, expansionHub;
    public LynxModule.BulkData controlHubData, expansionHubData;

    int startHangTicks = 0, startLiftTicks = 0, startExtendoTicks = 0;

    static BulkReader instance;
    public static BulkReader getInstance() {
        return instance;
    }
    public BulkReader(HardwareMap hardwareMap, boolean reset) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        if (allHubs.get(0).isParent()) {
            controlHub = allHubs.get(0);
            expansionHub = allHubs.get(1);
        } else {
            controlHub = allHubs.get(1);
            expansionHub = allHubs.get(0);
        }

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        BulkReader.instance = this;

        read();
        if (reset) {
            resetTicks();
        }
    }

    public BulkReader(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }


    public void resetTicks() {
        startExtendoTicks = getExtendoTicks();
        startHangTicks = getHangTicks();
        startLiftTicks = getLiftTicks();
    }

    public void read() {
        controlHubData = controlHub.getBulkData();
        expansionHubData = expansionHub.getBulkData();
    }

    public int getHangTicks() {
//        return controlHubData.getMotorCurrentPosition(-1) - startHangTicks;
        return 0;
    }

    public int getLiftTicks() {
        return expansionHubData.getMotorCurrentPosition(1) - startLiftTicks;
    }

    public int getExtendoTicks() {
        return expansionHubData.getMotorCurrentPosition(0) - startExtendoTicks;
    }
}
