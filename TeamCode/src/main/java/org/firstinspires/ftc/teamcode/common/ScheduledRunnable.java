package org.firstinspires.ftc.teamcode.common;

public class ScheduledRunnable {
    private final Runnable runnable;
    private final long delay;
    private final String type;
    private final long startTime;

    public ScheduledRunnable(Runnable runnable, long delay, String type) {
        this.runnable = runnable;
        this.delay = delay;
        this.type = type;
        this.startTime = System.currentTimeMillis();
    }

    public ScheduledRunnable(Runnable runnable, long delay) {
        this.runnable = runnable;
        this.delay = delay;
        this.type = "";
        this.startTime = System.currentTimeMillis();
    }

    public void run() {
        runnable.run();
    }

    public long getDelay() {
        return delay;
    }

    public String getType() {
        return type;
    }

    public long getStartTime() {
        return startTime;
    }

    public boolean isDue() {
        return System.currentTimeMillis() - startTime >= delay;
    }
}
