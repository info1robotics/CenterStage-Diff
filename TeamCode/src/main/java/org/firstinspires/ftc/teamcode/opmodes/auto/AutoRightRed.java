package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_RED;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.common.AutoUtil.*;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.common.ScheduledRunnable;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.enums.TSEPosition;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.followers.VariableHolonomicPIDVAFollower;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.ArrayList;

@Autonomous(name = "Red Right")
public class AutoRightRed extends AutoBase {
    static Pose2d startPose = p(TILE_SIZE * 0.5 + 3.15, -TILE_SIZE * 3 + 18, rad(90));

    MarkerCallback dropDetectionSequence = () -> {
        actionQueue.add(new ScheduledRunnable(pivot::setScore, 0, "pivot"));
        actionQueue.add(new ScheduledRunnable(joint::setTransition, 0, "joint"));
        actionQueue.add(new ScheduledRunnable(() -> {
            diffy.targetTicks.setLift(Lift.FIRST_PIXEL_DROP);
        }, 200, "lift"));
        actionQueue.add(new ScheduledRunnable(joint::setScore, 300, "joint"));
    };


    @Override
    public void onInit() {
        drive.setPoseEstimate(startPose);
        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(v(15.0, -26), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.33, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(v(52, -21.0), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, dropDetectionSequence)
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .addSpatialMarker(v(27.5, -20), () -> {
                    intake.setPower(1);
                })
                .splineTo(v(32.5, -20), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(v(50, -28.5), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, dropDetectionSequence)
                .build();

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(p(25.5, -40.5, HEADING_TO_RED), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.47, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.3, () -> {
                    intake.setPower(0);
                })
                .splineToSplineHeading(p(52, -35.8, HEADING_TO_BACKDROP), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, dropDetectionSequence)
                .build();

        Pose2d[] detectionEnds = {
                detectionLeft.end(),
                detectionMid.end(),
                detectionRight.end()
        };

        ArrayList<TrajectorySequence> cyclesTrajectories = new ArrayList<>();


        for (int i = 0; i < 3; i++) {
            int finalI = i;
            TrajectorySequenceBuilder cyclesBuilder = drive.trajectorySequenceBuilder(detectionEnds[i]);
            int NUM_CYCLES = 3;
            for (int j = 0; j < NUM_CYCLES; j++) {
                cyclesBuilder.setReversed(true)
                        .run(() -> {
                            diffy.targetTicks.setLift(Differential.LIFT_BOUND[0]);
                            pivot.setCollect();
                            joint.setCollect();
                            actionQueue.add(new ScheduledRunnable(cover::close, 500, "cover"));
                        })
                        .splineTo(v(17, -5), rad(180))
                        .run(() -> {
                            diffy.targetTicks.setExtendo(Differential.EXTENDO_BOUND[1]);
                            fold.setInit();
                        })
                        .run(() -> {
                            VariableHolonomicPIDVAFollower.setVariableTimeout(5);
                        })
                        .splineToConstantHeading(v(0, -6), rad(180))
                        .relativeTemporalMarker(0.3, () -> {
                            VariableHolonomicPIDVAFollower.setVariableTimeout(0.5);
                            intake.setPower(1);
                        })
                        .waitSeconds(0.5)
                        .relativeTemporalMarker(0.1, () -> {
                            fold.setPosition(Fold.autoPositions[finalI]);
                        })
                        .relativeTemporalMarker(0.3, () -> {
                            fold.setPosition(Fold.autoPositions[finalI + 1]);
                        })
                        .relativeTemporalMarker(0.5, () -> {
                            fold.setInit();
                            claw.close();
                            cover.open();
                            diffy.targetTicks.setExtendo(Differential.EXTENDO_BOUND[0]);
                        })
                        .relativeTemporalMarker(0.7, () -> {
                            intake.setPower(-1);
                        })
                        .relativeTemporalMarker(2, () -> {
                            intake.setPower(0);
                        });

                // backdrop sequence
                cyclesBuilder.setReversed(false)
                        .lineToSplineHeading(p(20, -2, HEADING_TO_BACKDROP))
                        .run(() -> {
                            actionQueue.add(new ScheduledRunnable(pivot::setScore, 0, "pivot"));
                            actionQueue.add(new ScheduledRunnable(joint::setTransition, 0, "joint"));
                            actionQueue.add(new ScheduledRunnable(() -> {
                                diffy.targetTicks.setLift(Lift.RANDOM_PIXEL_DROP);
                            }, 200, "lift"));
                            actionQueue.add(new ScheduledRunnable(joint::setScore, 300, "joint"));
                        })
                        .splineToConstantHeading(v(52.3, -30), Math.toRadians(HEADING_TO_BACKDROP))
                        .run(claw::open)
                        .waitSeconds(0.2);
            }
            cyclesTrajectories.add(cyclesBuilder.build());
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(cyclesTrajectories.get(0).end())
                .splineToSplineHeading(p(25.5, -40.5, HEADING_TO_RED), rad(HEADING_TO_BACKDROP))
                .build();

        startPos = AutoStartPos.RED_RIGHT;
        task = serial(
                conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(detectionRight)),

                serial(
                        execute(claw::open),
                        sleepms(540),
                        execute(() -> diffy.targetTicks.setLift(Differential.LIFT_BOUND[0]))
                ),
                conditional(() -> full, serial(
                        conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(cyclesTrajectories.get(0))),
                        conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(cyclesTrajectories.get(1))),
                        conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(cyclesTrajectories.get(2)))
                )),
                trajectorySequence(park)
        );
    }
}
