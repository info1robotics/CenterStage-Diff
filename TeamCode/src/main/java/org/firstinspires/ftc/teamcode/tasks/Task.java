package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

public abstract class Task {
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    public boolean isRunning() {
        return state == State.RUNNING;
    }

    public enum State {
        DEFAULT,
        RUNNING,
        FINISHED,
    }

    public State state = State.DEFAULT;
    public AutoBase context;

    public void run() {}
    public void tick() {}

    public void start(AutoBase context) {
        this.context = context;
        state = State.RUNNING;
        run();
    }
}