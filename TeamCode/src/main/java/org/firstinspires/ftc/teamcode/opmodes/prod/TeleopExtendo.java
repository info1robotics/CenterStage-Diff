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

@TeleOp(name = "teleop extendo")
public class TeleopExtendo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        BulkReader bulkReader = new BulkReader(this.hardwareMap);
        Differential diffy = new Differential(this.hardwareMap);

        Drivetrain drive = new Drivetrain(this.hardwareMap);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                drive.driveMecanum(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger, 1);
                gamepadEx1.update();
            }
        }).start();

        double hangPower, liftPower, extendoPower;

        while (opModeIsActive() && !isStopRequested()) {
            hangPower = 0;
            liftPower = 0;
            extendoPower = 0;

            bulkReader.read();
            diffy.reset();


            if (gamepad2.dpad_up) {
                extendoPower = 1;
            } else if (gamepad2.dpad_down) {
                extendoPower = -1;
            }

            liftPower = -gamepad2.left_stick_y;

            diffy.tick(hangPower, liftPower, extendoPower);
            diffy.update();

            gamepadEx2.update();
            log.tick();
        }
    }
}
