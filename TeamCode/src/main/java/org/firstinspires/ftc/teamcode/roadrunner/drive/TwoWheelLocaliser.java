package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private Encoder parallelEncoder, perpEncoder;

    public TwoWheelLocaliser(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(-2.78, -6.88, 0), // right
                new Pose2d(5.37, 0, Math.toRadians(90)) // front
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL")); // parallel
        perpEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL")); // perpendicular
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int rightPos = parallelEncoder.getCurrentPosition();
        int frontPos = perpEncoder.getCurrentPosition();

        return Arrays.asList(
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        int rightVel = (int) parallelEncoder.getCorrectedVelocity();
        int frontVel = (int) perpEncoder.getCorrectedVelocity();


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
