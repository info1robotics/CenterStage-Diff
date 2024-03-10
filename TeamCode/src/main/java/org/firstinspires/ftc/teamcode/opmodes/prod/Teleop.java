package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.BulkReader;
import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.differential.Differential;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@TeleOp(name = "teleop pur teoretic smr mama")
public class Teleop extends LinearOpMode {
    public static final double liftOpenCoverTick = 2000;
    public static final double liftCloseCoverTick = 1000;

    public static final double liftCollectTick = 4000;
    public static final double liftScoreTick = 20000;

    public static final double waitForCoverMs = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        Differential diffy = new Differential(this.hardwareMap);

        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Pivot pivot = new Pivot(this.hardwareMap);
        Claw claw = new Claw(this.hardwareMap);
        Cover cover = new Cover(this.hardwareMap);
        Fold fold = new Fold(this.hardwareMap);
        Intake intake = new Intake(this.hardwareMap);

        fold.setDrive();
        cover.setClosed();
        claw.open();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
//                if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                drive.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger, 1);
//                }
                gamepadEx1.update();
            }
        }).start();

        double hangPower, liftPower, exendoPower;

        long timeLastCoverWait = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {
            hangPower = 0;
            liftPower = 0;
            exendoPower = 0;

            bulkReader.read();
            diffy.reset();

            double leftStickY = -gamepad2.left_stick_y;
            double rightStickY = -gamepad2.right_stick_y;

            if (leftStickY > 0.06) {
                intake.setPower(Math.min(leftStickY, 0.4));
            } else if (leftStickY < 0.06) {
                intake.setPower(Math.max(-0.4, leftStickY));
            }

            if (rightStickY > 0.06) {
                liftPower = Math.max(rightStickY, 0.5);
            } else if (rightStickY < 0.06) {
                liftPower = Math.min(-0.5, rightStickY);
            }

            if (gamepad2.dpad_up) {
                exendoPower = 0.5;
            } else if (gamepad2.dpad_down) {
                exendoPower = -0.5;
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

            if (bulkReader.getLiftTicks() > -liftOpenCoverTick && liftPower > 0 && cover.isClosed()) {
                cover.setOpen();
                liftPower = 0;
                timeLastCoverWait = System.currentTimeMillis();
            }

            if (timeLastCoverWait - System.currentTimeMillis() < waitForCoverMs) {
                liftPower = 0;
            }

            if (liftPower < 0 && bulkReader.getLiftTicks() > -liftCloseCoverTick) {
                cover.setClosed();
            }

            if (bulkReader.getLiftTicks() > -liftCollectTick) {
                pivot.setCollect();
            } else if (bulkReader.getLiftTicks() < -liftScoreTick) {
                pivot.setScore();
            }

            diffy.tick(hangPower, liftPower, exendoPower);
            diffy.update();

            gamepadEx2.update();
            log.tick();
        }
    }
}
