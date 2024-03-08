package org.firstinspires.ftc.teamcode.differential;

public class DifferentialTickBuffer {
    LimitedQueue<Double> qExtendo, qLift, qHang;

    public DifferentialTickBuffer() {
        qExtendo = new LimitedQueue<>(100);
        qLift = new LimitedQueue<>(100);
        qHang = new LimitedQueue<>(100);
    }

    public void addTicks(double extendo, double lift, double hang) {
        qExtendo.add(extendo);
        qLift.add(lift);
        qHang.add(hang);
    }

    public boolean isExtendoStable() {
        return isStable(qExtendo);
    }

    public boolean isLiftStable() {
        return isStable(qLift);
    }

    public boolean isHangStable() {
        return isStable(qHang);
    }

    public boolean isStable(LimitedQueue<Double> q) {
        double avg = q.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        return Math.abs(avg - q.get(q.size() - 1)) < 50;
    }
}
