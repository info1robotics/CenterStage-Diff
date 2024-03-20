package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.ActionQueue;
import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.common.ScheduledRunnable;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Joint;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Autonomous
public class LiftTarget extends LinearOpMode {
    ActionQueue actionQueue = new ActionQueue();
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        BulkReader reader = new BulkReader(hardwareMap);
        Differential diffy = new Differential(hardwareMap);
        Pivot pivot = new Pivot(this.hardwareMap);
        Joint joint = new Joint(this.hardwareMap);
        Claw claw = new Claw(this.hardwareMap);


        Runnable dropDetectionSequence = () -> {
            actionQueue.add(new ScheduledRunnable(pivot::setScore, 0, "pivot"));
            actionQueue.add(new ScheduledRunnable(joint::setTransition, 30, "joint"));
            actionQueue.add(new ScheduledRunnable(() -> {
                diffy.userTargetPosition.setLift(Lift.FIRST_PIXEL_DROP);
            }, 200, "lift"));
            actionQueue.add(new ScheduledRunnable(joint::setScore, 300, "joint"));
        };

        claw.close();
        waitForStart();
        dropDetectionSequence.run();
        joint.setTransition();
//        diffy.targetTicks.setLift(Lift.FIRST_PIXEL_DROP);
        while (opModeIsActive()) {
            reader.read();
            diffy.reset();

            diffy.tick(0, 0, 0);
            diffy.update();
            actionQueue.tick();
            log.tick();
        }
    }
}
