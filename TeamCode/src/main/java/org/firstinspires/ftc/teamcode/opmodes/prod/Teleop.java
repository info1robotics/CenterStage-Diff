package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.ActionQueue;
import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.common.ScheduledRunnable;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Joint;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@TeleOp(name = "teleop pur teoretic smr mama")
public class Teleop extends LinearOpMode {

    public static final long IGNORE_LIFT_FOR_MS = 300;
    private long msUntilIgnoreLift = System.currentTimeMillis();

    ActionQueue actionQueue = new ActionQueue();

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        BulkReader bulkReader = new BulkReader(this.hardwareMap, false);
        Differential diffy = new Differential(this.hardwareMap);

        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Pivot pivot = new Pivot(this.hardwareMap);
        Claw claw = new Claw(this.hardwareMap);
        Cover cover = new Cover(this.hardwareMap);
        Fold fold = new Fold(this.hardwareMap);
        Intake intake = new Intake(this.hardwareMap);
        Joint joint = new Joint(this.hardwareMap);

        fold.setDrive();
        cover.close();
        pivot.setCollect();
        joint.setCollect();
        claw.open();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                drive.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger, 1);
                }
                if (gamepad1.left_bumper) {
                    drive.driveMecanum(0, -1, 0, 0.5);
                } else if (gamepad1.right_bumper) {
                    drive.driveMecanum(0, 1, 0, 0.5);
                }
                gamepadEx1.update();
            }
        }).start();

        double hangPower, liftPower, exendoPower;

        while (opModeIsActive() && !isStopRequested()) {
            hangPower = 0;
            liftPower = 0;
            exendoPower = 0;

            bulkReader.read();
            diffy.reset();

            double leftStickY = -gamepad2.left_stick_y;
            double rightStickY = -gamepad2.right_stick_y;

            if (leftStickY > 0.06) {
                intake.setPower(Math.max(leftStickY, 0.7));
            } else if (leftStickY < -0.06) {
                intake.setPower(Math.min(-0.7, leftStickY));
            } else {
                intake.setPower(0.0);
            }

            if (rightStickY > 0.06) {
                liftPower = Math.max(rightStickY, 0.5);
            } else if (rightStickY < -0.06) {
                liftPower = Math.min(-0.5, rightStickY);
            }

            log.add("Right Stick Y", rightStickY);

            if (gamepad2.start) {
                // sign = + if dpad up else - dpad down
                int sign = gamepadEx2.getButtonDown("dpad_up") ? 1 : (gamepadEx2.getButtonDown("dpad_down") ? -1 : 0);
                diffy.targetTicks.setExtendo(diffy.targetTicks.getExtendo() + sign * 5000);
            }

            if (gamepadEx2.getButtonDown("bumper_left")) {
                claw.toggle(Claw.Side.LEFT);
            }
            if (gamepadEx2.getButtonDown("bumper_right")) {
                claw.toggle(Claw.Side.RIGHT);
            }


            if (gamepadEx2.getButtonDown("a")) {
                claw.open();
            } else if (gamepadEx2.getButtonDown("b")) {
                claw.close();
            }

            if (System.currentTimeMillis() < msUntilIgnoreLift) {
                liftPower = 0;
            }

            if (liftPower > 0.1) {
                if (BulkReader.getInstance().getLiftTicks() > -2300 && !pivot.is(Pivot.PivotState.SCORE)) {
                    actionQueue.clear();
                    long delay = cover.isClosed() ? IGNORE_LIFT_FOR_MS : 0;
                    if (cover.isClosed()) {
                        cover.open();
                        msUntilIgnoreLift = System.currentTimeMillis() + IGNORE_LIFT_FOR_MS;
                    }

                    claw.close();
                    actionQueue.add(new ScheduledRunnable(pivot::setScore, delay, "pivot"));
                    actionQueue.add(new ScheduledRunnable(joint::setTransition, delay, "joint"));
                    actionQueue.add(new ScheduledRunnable(joint::setScore, 300 + delay, "joint"));
                    actionQueue.add(new ScheduledRunnable(cover::close, 800 + delay, "cover"));
                }
            } else if (liftPower < -0.1) {
                if (BulkReader.getInstance().getLiftTicks() > -15000 && !pivot.is(Pivot.PivotState.COLLECT)) {
                    actionQueue.clear();
                    if (cover.isClosed()) {
                        cover.open();
                    }

                    actionQueue.add(new ScheduledRunnable(pivot::setCollect, 0, "pivot"));
                    actionQueue.add(new ScheduledRunnable(joint::setCollect, 0, "joint"));
                    actionQueue.add(new ScheduledRunnable(cover::close,  800, "cover"));
                }
            }

            diffy.tick(hangPower, liftPower, exendoPower);
            diffy.update();

            gamepadEx2.update();
            actionQueue.tick();
            log.tick();
        }
    }
}
