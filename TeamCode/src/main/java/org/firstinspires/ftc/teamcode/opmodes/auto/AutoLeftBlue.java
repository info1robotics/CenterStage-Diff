package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BLUE;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_RED;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.common.AutoUtil.p;
import static org.firstinspires.ftc.teamcode.common.AutoUtil.rad;
import static org.firstinspires.ftc.teamcode.common.AutoUtil.v;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.conditional;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectory;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.ScheduledRunnable;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.enums.TSEPosition;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.ArrayList;

@Autonomous(name = "Blue Left")
public class AutoLeftBlue extends AutoBase {
    static Pose2d startPose = p(TILE_SIZE * 0.5 + 3.15, TILE_SIZE * 3 - 18, rad(-91.5));

    void detectionSeq() {
        actionQueue.add(new ScheduledRunnable(pivot::setScore, 0, "pivot"));
        actionQueue.add(new ScheduledRunnable(joint::setTransition, 30, "joint"));
        actionQueue.add(new ScheduledRunnable(() -> {
            diffy.userTargetPosition.setLift(Lift.FIRST_PIXEL_DROP);
        }, 200, "lift"));
        actionQueue.add(new ScheduledRunnable(joint::setScore, 300, "joint"));
    }


    MarkerCallback dropDetectionSequence = this::detectionSeq;


    @Override
    public void onInit() {
        drive.setPoseEstimate(startPose);
        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(p(30, 17.5, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.03, () -> {
                    intake.setPower(-0.5);
                })
                .waitSeconds(0.15)
                .relativeTemporalMarker(0.3, this::detectionSeq)
                .relativeTemporalMarker(0.4, () -> {
                    intake.setPower(0);
                })
                .waitSeconds(0.4)
                .splineToConstantHeading(v(49, 36.0), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.1, claw::open)
                .waitSeconds(0.1)
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(p(30, 14.5, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.3, () -> intake.setPower(-0.4))
                .waitSeconds(0.1)
                .lineToLinearHeading(p(23, 14.5, HEADING_TO_BACKDROP))
                .waitSeconds(0.1)
                .run(this::detectionSeq)
                .relativeTemporalMarker(0.2, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(v(49, 29.3), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.1, claw::open)
                .waitSeconds(0.1)
                .build();

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(v(12.3, 26), rad(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.33, () -> {
                    intake.setPower(-0.4);
                })
                .relativeTemporalMarker(0.6, intake::stop)
                .waitSeconds(0.2)
                .lineToConstantHeading(v(48.4, 22))
                .relativeTemporalMarker(-1.6, dropDetectionSequence)
                .relativeTemporalMarker(-0.2, claw::open)
                .build();

        Pose2d[] detectionEnds = {
                detectionLeft.end(),
                detectionMid.end(),
                detectionRight.end()
        };

        ArrayList<TrajectorySequence> cyclesTrajectories = new ArrayList<>();

        Differential.EXTENDO_BOUND[0] -= 4000;

        for (int i = 0; i < 3; i++) {
            TrajectorySequenceBuilder cyclesBuilder = drive.trajectorySequenceBuilder(detectionEnds[i]);
            int NUM_CYCLES = 2;
            for (int j = 0; j < NUM_CYCLES; j++) {
                int finalJ = j;
                cyclesBuilder.setReversed(true)
                        .run(() -> {
                            diffy.userTargetPosition.setLift(-1);
                            diffy.targetTicks.setLift(Differential.LIFT_BOUND[1]);
                            pivot.setTransition();
                            joint.setCollect();
                            actionQueue.add(new ScheduledRunnable(cover::close, 500, "cover"));
                        })
                        .splineToSplineHeading(p(17, 49.5, HEADING_TO_BACKDROP), rad(180))
                        .splineToSplineHeading(p(-24.85, 49.5, HEADING_TO_BACKDROP + rad(28)), rad(180))
                        .relativeTemporalMarker(0, () -> {
                            diffy.targetTicks.setExtendo(Differential.EXTENDO_BOUND[1]);
                            fold.setPosition(Fold.FOLD_UP);
                            intake.setPower(1);
                        })
                        .run(() -> {
                            long startDelay = 1000;
                            actionQueue.add(new ScheduledRunnable(() -> {
                                fold.setPosition(Fold.autoPositions[finalJ]);
                            }, startDelay, "firstFold"));
                            actionQueue.add(new ScheduledRunnable(() -> {
                                fold.setPosition(Fold.autoPositions[finalJ + 1]);
                            }, startDelay + 450, "secondFold"));
                            actionQueue.add(new ScheduledRunnable(intake::reverse, startDelay + 800));
                            actionQueue.add(new ScheduledRunnable(intake::take, startDelay + 900));
                            actionQueue.add(new ScheduledRunnable(() -> {
                                fold.setPosition(Fold.FOLD_UP);
                                claw.open();
                                diffy.targetTicks.setExtendo(Differential.EXTENDO_BOUND[0]);
                            }, startDelay + 1500, "finalStack"));
                            actionQueue.add(new ScheduledRunnable(pivot::setCollect, startDelay + 2700));
                            actionQueue.add(new ScheduledRunnable(claw::close, startDelay + 2900));
                            actionQueue.add(new ScheduledRunnable(cover::open, startDelay + 3300));
                        })
                        .waitSeconds(3.3);

                // backdrop sequence
                cyclesBuilder.setReversed(false)
                        .run(intake::reverse)
                        .lineToSplineHeading(p(-24, 50, HEADING_TO_BACKDROP))
                        .lineToSplineHeading(p(17, 50, HEADING_TO_BACKDROP))
                        .relativeTemporalMarker(-0.7, intake::stop)
                        .relativeTemporalMarker(-0.5, () -> {
                            actionQueue.add(new ScheduledRunnable(pivot::setScore, 0, "pivot"));
                            actionQueue.add(new ScheduledRunnable(joint::setTransition, 0, "joint"));
                            actionQueue.add(new ScheduledRunnable(() -> {
                                diffy.userTargetPosition.setLift(Lift.RANDOM_PIXEL_DROP);
                            }, 200, "lift"));
                            actionQueue.add(new ScheduledRunnable(joint::setScore, 300, "joint"));
                        })
                        .splineToConstantHeading(v(46.0, 30), Math.toRadians(HEADING_TO_BACKDROP))
                        .waitSeconds(0.2)
                        .run(claw::open)
                        .waitSeconds(0.5);
            }
            cyclesTrajectories.add(cyclesBuilder.build());
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(cyclesTrajectories.get(0).end())
                .run(() -> {
                    diffy.userTargetPosition.setLift(-1);
                    diffy.targetTicks.setLift(Differential.LIFT_BOUND[1]);
                    pivot.setCollect();
                    joint.setCollect();
                })
//                .splineToSplineHeading(p(40.5, 45.5, -HEADING_TO_BLUE), rad(-HEADING_TO_BACKDROP))
                .lineTo(v(40.5,54.5))
                .build();

        startPos = AutoStartPos.BLUE_LEFT;
        task = serial(
                conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(detectionRight)),
                conditional(() -> full, serial(
                        conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(cyclesTrajectories.get(0))),
                        conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(cyclesTrajectories.get(1))),
                        conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(cyclesTrajectories.get(2)))
                )),
                trajectorySequence(park)
        );
    }
}
