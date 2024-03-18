package org.firstinspires.ftc.teamcode.tasks;


import java.util.function.Supplier;

public class ConditionalTask extends Task {
    Supplier<Boolean> condition;
    public Task task;

    public ConditionalTask(Supplier<Boolean> condition, Task task) {
        this.condition = condition;
        this.task = task;
    }

    @Override
    public void tick() {
        if (task.isFinished()) {
            state = State.FINISHED;
            return;
        }
        task.tick();
    }

    @Override
    public void run() {
        if (!condition.get()) {
            state = State.FINISHED;
            return;
        }
        task.start(this.context);
    }
}
