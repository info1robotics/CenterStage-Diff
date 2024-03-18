package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Log {
    Telemetry telemetry;
    public static Log instance;

    public static Log getInstance() {
        return instance;
    }

    public Log(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        instance = this;
    }

    public Log add(String name, Object line) {
        telemetry.addData(name, line);
        return this;
    }

    public Log add(String line) {
        telemetry.addLine(line);
        return this;
    }

    public void tick() {
        telemetry.update();
    }

    public Log clear() {
        telemetry.clearAll();
        return this;
    }
}
