package org.firstinspires.ftc.teamcode.tasks;

/**
 * Runs the given block synchronously.
 * Intended for one-liners.
 */
public class ExecuteTask extends Task {
    private final Runnable runnable;
    public ExecuteTask(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void run() {
        runnable.run();
        state = State.FINISHED;
    }
}
