package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Collections;

public final class TrajectorySequenceSegment extends SequenceSegment {
    private final TrajectorySequence trajectorySequence;

    public TrajectorySequenceSegment(TrajectorySequence trajectory) {
        // Note: Markers are already stored in the `Trajectory` itself.
        // This class should not hold any markers
        super(trajectory.duration(), trajectory.start(), trajectory.end(), Collections.emptyList());
        this.trajectorySequence = trajectory;
    }

    public TrajectorySequence getTrajectorySequence() {
        return this.trajectorySequence;
    }
}
