package org.firstinspires.ftc.teamcode.opmodes.debug;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class CameraTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        TSEDetectionPipeline pipeline = new TSEDetectionPipeline(AutoStartPos.BLUE_RIGHT);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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
        android.util.Log.d("NICHITA", "hiii");

        while (!isStarted() && !isStopRequested()) {
            log.add(pipeline.getAnalysis().toString());
            log.add(Integer.toString(pipeline.s1));
            log.add(Integer.toString(pipeline.s2));
            log.add(Integer.toString(pipeline.s3));
            log.tick();
        }
    }
}