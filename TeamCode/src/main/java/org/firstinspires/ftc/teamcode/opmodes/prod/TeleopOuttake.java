package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Joint;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@TeleOp
public class TeleopOuttake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(this.telemetry);
        Pivot pivot = new Pivot(this.hardwareMap);
        Claw claw = new Claw(this.hardwareMap);
        Cover cover = new Cover(this.hardwareMap);
        Joint joint = new Joint(this.hardwareMap);

        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        cover.setOpen();
        claw.open();
        pivot.setCollect();
        joint.setCollect();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepadEx2.getButtonDown("a")) {
                pivot.setCollect();
            } else if (gamepadEx2.getButtonDown("y")) {
                pivot.setScore();
            }

            if (gamepadEx2.getButtonDown("x")) {
                joint.setCollect();
            } else if (gamepadEx2.getButtonDown("b")) {
                pivot.setScore();
            }

            if (gamepadEx2.getButtonDown("bumper_left")) {
                claw.toggle(Claw.Side.LEFT);
            } else if (gamepadEx2.getButtonDown("bumper_right")) {
                claw.toggle(Claw.Side.RIGHT);
            }

            log.tick();
            gamepadEx2.update();
        }
    }
}
