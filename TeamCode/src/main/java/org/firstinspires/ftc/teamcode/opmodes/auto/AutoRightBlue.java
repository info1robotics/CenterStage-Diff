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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "Blue Right")
public class AutoRightBlue extends AutoBase {
    static Pose2d startPose = p(-TILE_SIZE  - 14, TILE_SIZE * 3 - 13, Math.toRadians(-91.5));
    public static final long IGNORE_LIFT_FOR_MS = 300;
    private long msUntilIgnoreLift = System.currentTimeMillis();
    void detectionSeq() {
        claw.close();
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
        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.1)
                .forward(10)
                .lineToLinearHeading(new Pose2d(-38,25,Math.toRadians(140)))
                .strafeRight(10)
                /*
                .lineToLinearHeading(new Pose2d(-TILE_SIZE-13,27-TILE_SIZE+28.5,Math.toRadians(HEADING_TO_BACKDROP)))
                //tune
                .run(()->{intake.setPower(-0.8);})
                .waitSeconds(0.15)
                .relativeTemporalMarker(0.3, this::detectionSeq)
                .run(()->{intake.setPower(0);})
                .lineToLinearHeading(new Pose2d(-TILE_SIZE-13, 27-TILE_SIZE+8.5,HEADING_TO_BACKDROP))
                .waitSeconds(0.4)
                .forward(50)
                .splineToConstantHeading(new Vector2d(49, 28.0), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.1, claw::open)
                .waitSeconds(30)

                 */


                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .forward(TILE_SIZE+3)
                .back(TILE_SIZE)
                //.strafeLeft(TILE_SIZE*3+13)
                /*

                .lineToConstantHeading(new Vector2d(-TILE_SIZE-13, 27-TILE_SIZE+7.5))

                .run(()->{intake.setPower(-0.8);})



                .turn(rad(90))
                .forward(40)

                .splineTo(new Vector2d(50.5,34.9), Math.toRadians(HEADING_TO_BACKDROP))
                .run(()->{intake.setPower(0);})
                .waitSeconds(0.2)
                .run(this::detectionSeq)
                .waitSeconds(0.1)
                .run(claw::open)
                /*
                .relativeTemporalMarker(0.1, claw::open)

                 */


                .waitSeconds(30)


                .build();

        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose )
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-30.5,26,Math.toRadians(20)))
                .turn(Math.toRadians(-20))
                .strafeRight(15)
                ////.strafeLeft(TILE_SIZE*3+13)
                /*
                .forward(10)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE-13, 27, Math.toRadians(180)))

                .run(()->{intake.setPower(-0.8);})
                .strafeLeft(TILE_SIZE)

                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(50.3,39.5), Math.toRadians(HEADING_TO_BACKDROP))


                .relativeTemporalMarker(-1.6, dropDetectionSequence)

                .relativeTemporalMarker(-0.35, claw::open)
                /*

                .forward(20)
                .lineToConstantHeading(v(48.4, 22))
                .relativeTemporalMarker(-1.6, dropDetectionSequence)
                .relativeTemporalMarker(-0.2, claw::open)
                 */


                .waitSeconds(30)
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
                        .splineToSplineHeading(p(-54, 49.5, HEADING_TO_BACKDROP + rad(28)), rad(180))
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

        startPos = AutoStartPos.BLUE_RIGHT;
        task = serial(
                conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(detectionRight))
//                conditional(() -> full, serial(
//                        conditional(() -> tsePosition == TSEPosition.LEFT, trajectorySequence(cyclesTrajectories.get(0))),
//                        conditional(() -> tsePosition == TSEPosition.CENTER, trajectorySequence(cyclesTrajectories.get(1))),
//                        conditional(() -> tsePosition == TSEPosition.RIGHT, trajectorySequence(cyclesTrajectories.get(2)))
//                )),
                //trajectorySequence(park)
        );





    }
}
