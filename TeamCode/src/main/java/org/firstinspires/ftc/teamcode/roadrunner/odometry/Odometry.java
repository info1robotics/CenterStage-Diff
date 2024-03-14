package org.firstinspires.ftc.teamcode.roadrunner.odometry;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.common.BulkReader;

public class Odometry {
    private final int port;
    private final boolean reversed;
    private double[] velocityEstimates;
    private final static int CPS_STEP = 0x10000;
    private int lastPosition;
    private NanoClock clock;
    private double lastUpdateTime;
    private int velocityEstimateIdx;

    public Odometry(int port, boolean reversed) {
        this.port = port;
        this.reversed = reversed;

        this.clock = NanoClock.system();

        this.lastPosition = 0;
        this.velocityEstimates = new double[3];
        this.lastUpdateTime = clock.seconds();
    }

    public int getCurrentPosition() {
        int currentPosition = BulkReader.getInstance().controlHubData.getMotorCurrentPosition(port) * (reversed ? -1 : 1);
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        return BulkReader.getInstance().controlHubData.getMotorVelocity(port) * (reversed ? -1 : 1);
    }
    private static double inverseOverflow(double input, double estimate) {
        int real = (int) input & 0xffff;
        real += ((real % 20) / 4) * CPS_STEP;
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }
}
