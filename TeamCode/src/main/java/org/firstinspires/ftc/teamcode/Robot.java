package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.opmodes.subsystems.Drivetrain;

import java.util.List;

public class Robot {
    private static Robot instance;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public Drivetrain drivetrain;

    public AutoStartPos autoStartPos = AutoStartPos.UNKNOWN;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        drivetrain = new Drivetrain(hardwareMap);


        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    public boolean isValid() {
        return hardwareMap != null &&
                telemetry != null &&
                autoStartPos != AutoStartPos.UNKNOWN;
    }

    public void initDetection() {
//        OpenCvPipeline detectionPipeline = null;
//        switch (autoPosition) {
//            case RED_LEFT:
//                detectionPipeline = new TSEDetectionPipelineLeftRed();
//            case RED_RIGHT:
//                detectionPipeline = new TSEDetectionPipelineRightRed();
//                break;
//            case BLUE_LEFT:
//                detectionPipeline = new TSEDetectionPipelineLeftBlue();
//                break;
//            case BLUE_RIGHT:
//                detectionPipeline = new TSEDetectionPipelineRightBlue();
//                break;
//        }
//
//        assert detectionPipeline != null;
//        vision = new Vision(hardwareMap, "Webcam 1", 640, 360, OpenCvCameraRotation.UPRIGHT, detectionPipeline);
//        vision.openCamera();
    }


    public void initAuto(AutoStartPos autoStartPos) {
        this.autoStartPos = autoStartPos;
        Robot.getInstance().telemetry = new MultipleTelemetry((Robot.getInstance().telemetry), FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        initDetection();
    }

    public void initTele() {
        this.autoStartPos = AutoStartPos.TELE;
    }
}
