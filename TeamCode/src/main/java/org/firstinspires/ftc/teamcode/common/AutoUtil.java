package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutoUtil {
    public static Vector2d v(double x, double y) {
        return new Vector2d(x, y);
    }

    public static Pose2d p(double x, double y, double theta) {
        return new Pose2d(x, y, theta);
    }

    public static double rad(double angdeg) {
        return Math.toRadians(angdeg);
    }
}
