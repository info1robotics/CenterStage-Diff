package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.ActionQueue;
import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.enums.TSEPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Joint;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class AutoBase extends LinearOpMode {
    public static State state = State.DEFAULT;
    static AutoBase instance = null;
    public AutoStartPos startPos = AutoStartPos.UNKNOWN;//verify before match
    public TSEPosition tsePosition = TSEPosition.RIGHT;
    public OpenCvCamera camera;
    public TSEDetectionPipeline pipeline;
    public Pivot pivot;
    public Claw claw;
    public Fold fold;
    public Cover cover;
    public Joint joint;
    public Intake intake;
    public Drone drone;
    public Differential diffy;
    public SampleMecanumDrive drive;
    public Task task;

    public double extendoPower, liftPower, hangPower;

    public ActionQueue actionQueue = new ActionQueue();

    public boolean full = true;

    public static AutoBase getInstance() {
        return instance;
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        return result;
    }

    public void onInit() {
    }

    public void onInitTick() {
    }

    public void onStart() throws InterruptedException {
    }

    public void onStartTick() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        instance = this;
        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        pivot = new Pivot(this.hardwareMap);
        claw = new Claw(this.hardwareMap);
        intake = new Intake(this.hardwareMap);
        fold = new Fold(this.hardwareMap);
        joint = new Joint(this.hardwareMap);
        drone = new Drone(this.hardwareMap);
        cover = new Cover(this.hardwareMap);
        diffy = new Differential(this.hardwareMap);
        drive = new SampleMecanumDrive(this.hardwareMap);

        BulkReader.autoLiftTicks = bulkReader.startLiftTicks;
        BulkReader.autoExtendoTicks = bulkReader.startExtendoTicks;

//        fold.setInit();
        cover.open();
        joint.setCollect();
        pivot.setCollect();
        claw.close();
        //drone.setPosition(1);

        onInit();

        if (startPos == AutoStartPos.UNKNOWN) {
            log.add("Auto starting position not set.");
            log.tick();
            return;
        }

        state = State.INIT;
        enableVision();
        while (!isStarted() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();

            log.add("A/X for Full");
            log.add("B/O for Detection");

            if (gamepad1.a) {
                full = true;
            }
            if (gamepad1.b) {
                full = false;
            }
            if (full) {
                log.add("Running Full");
            } else {
                log.add("Detection Only");
            }

            tsePosition = pipeline.getAnalysis();
            log.add("POSITION",tsePosition.toString());

            log.tick();
            diffy.tick(0, 0, 0);
            diffy.update();
            onInitTick();
        }

        log.add("Final Detection", tsePosition.toString());
        log.tick();

        camera.closeCameraDeviceAsync(() -> {});
        onStart();

        fold.setPosition(Fold.FOLD_UP);

        state = State.START;
        if (task != null) task.start(this);

        while (opModeIsActive() && !isStopRequested()) {
            bulkReader.read();
            diffy.reset();

            if (task != null) task.tick();
            onStartTick();

            log.tick();
            actionQueue.tick();
            diffy.tick(hangPower, liftPower, extendoPower);
            diffy.update();
        }

    }

    public void enableVision() {
        pipeline = new TSEDetectionPipeline(startPos);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("camera ", cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeline);

        try {

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } catch (Exception e) {

        }

    }

    public enum State {
        DEFAULT,
        INIT,
        START
    }
}
