package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.enums.TSEPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TSEDetectionPipeline extends OpenCvPipeline {
    static final Scalar BLUE = new Scalar(0, 0, 255);
    public static boolean debug = false;
    static Scalar lowerBlue = new Scalar(0, 0, 60);
    static Scalar upperBlue = new Scalar(25, 187, 255);
    static Scalar lowerRed = new Scalar(50, 0, 0);
    static Scalar upperRed = new Scalar(180, 60, 60);
    private final AutoStartPos autoStartPos;
    Scalar lowerConstraint;
    Scalar upperConstraint;
    Rect left, center, right;
    Mat region1, region2, region3;
    private TSEPosition position = TSEPosition.LEFT;

    public TSEDetectionPipeline(AutoStartPos autoStartPos) {
        this.autoStartPos = autoStartPos;
        switch (autoStartPos) {
            case RED_LEFT:
                left = new Rect(114, 38, 239, 323);
                center = new Rect(266, 38, 225, 323);
                right = new Rect(508, 38, 239, 323);
                break;
            case RED_RIGHT:
                left = new Rect(1, 38, 125, 265);
                center = new Rect(171, 48, 225, 265);
                right = new Rect(448, 38, 125, 265);
                break;
            case BLUE_LEFT:
                left = new Rect(160, 38, 125, 285);
                center = new Rect(316, 38, 180, 285);
                right = new Rect(508, 38, 125, 285);
                break;
            case BLUE_RIGHT:
                left = new Rect(1, 38, 125, 265);
                center = new Rect(152, 38, 225, 265);
                right = new Rect(399, 38, 125, 265);
                break;
        }

        if (autoStartPos.isRed()) {
            lowerConstraint = lowerRed;
            upperConstraint = upperRed;
        } else {
            lowerConstraint = lowerBlue;
            upperConstraint = upperBlue;
        }
    }

    @Override
    public void init(Mat firstFrame) {
    }


    @Override
    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);

            if (!debug) {
                Core.inRange(input, lowerConstraint, upperConstraint, input);

                region1 = new Mat(input, left);
                region2 = new Mat(input, center);
                region3 = new Mat(input, right);

                int sel1 = Core.countNonZero(region1);
                int sel2 = (int) (Core.countNonZero(region2) * 1.25);
                int sel3 = Core.countNonZero(region3);
                if (autoStartPos == AutoStartPos.BLUE_LEFT) sel3 = 450;

                Log.getInstance()
                        .add("Pixels Left", sel1)
                        .add("Pixels Mid", sel2)
                        .add("Pixels Right", sel3);

                long max = Math.max(Math.max(sel1, sel2), sel3);

                if (sel1 == max) {
                    position = TSEPosition.LEFT;
                } else if (sel2 == max) {
                    position = TSEPosition.CENTER;
                } else if (sel3 == max) {
                    position = TSEPosition.RIGHT;
                }
                region1.release();
                region2.release();
                region3.release();
            } else {

                Imgproc.rectangle(input, left, BLUE, 2);
                Imgproc.rectangle(input, center, BLUE, 2);
                Imgproc.rectangle(input, right, BLUE, 2);
            }
            try {
                sleep(100);
            } catch (InterruptedException e) {
            }
        } catch (Exception e) {
        }

        return input;
    }

    public TSEPosition getAnalysis() {
        return position;
    }

}