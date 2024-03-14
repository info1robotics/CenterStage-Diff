package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class RRLocalizationTest extends LinearOpMode {
    long timeLastTick = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        double ileft = ((TwoWheelLocaliser) drive.getLocalizer()).getWheelPositions().get(0);
//        double iright = ((TwoWheelLocaliser) drive.getLocalizer()).getWheelPositions().get(1);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            bulkReader.read();
            telemetry.addData("Since Last Tick", System.currentTimeMillis() - timeLastTick);
            timeLastTick = System.currentTimeMillis();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            log
                    .add("x", poseEstimate.getX())
                    .add("y", poseEstimate.getY())
                    .add("heading", poseEstimate.getHeading());

            telemetry.update();
        }
    }
}
