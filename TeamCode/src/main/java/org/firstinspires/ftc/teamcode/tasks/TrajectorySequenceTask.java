package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceTask extends Task {

    TrajectorySequence trajectorySequence;

    public TrajectorySequenceTask(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;
    }
    @Override
    public void tick() {
        if (!Thread.currentThread().isInterrupted() && context.drive.isBusy()) {
            context.drive.update();
        }
        else state = State.FINISHED;
    }

    @Override
    public void run() {
        context.drive.followTrajectorySequenceAsync(trajectorySequence);
    }
}

