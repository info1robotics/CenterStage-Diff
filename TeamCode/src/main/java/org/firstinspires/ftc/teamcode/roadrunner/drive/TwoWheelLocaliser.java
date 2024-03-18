package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.roadrunner.odometry.Odometry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelLocaliser extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double X_MULTIPLIER = 1.006711409396 * 1.002227171492 * 0.9959057209;
    public static double Y_MULTIPLIER = 1.007432613952;
    private final SampleMecanumDrive drive;

    private final Odometry parallelOdo = new Odometry(3, false);
    private final Odometry perpendicularOdo = new Odometry(0, false);

    public TwoWheelLocaliser(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(6.65, 3.66675, 0), // left
                new Pose2d(6.25, -0.3, Math.toRadians(90)) // front
        ));

        this.drive = drive;
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int rightPos = parallelOdo.getCurrentPosition();
        int frontPos = perpendicularOdo.getCurrentPosition();

        Log.getInstance()
                .add("Right Odo Pos", rightPos)
                .add("Front Odo Pos", frontPos);

        return Arrays.asList(
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        int rightVel = (int) parallelOdo.getCorrectedVelocity();
        int frontVel = (int) perpendicularOdo.getCorrectedVelocity();


        return Arrays.asList(
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Nullable
    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }


}
