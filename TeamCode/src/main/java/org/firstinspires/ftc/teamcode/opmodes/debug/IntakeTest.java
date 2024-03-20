package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(group = "Debug")
@Config
public class IntakeTest extends LinearOpMode {
    public static double intakePower = 1;
    public static double foldPos = 0.7;


    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);
        Intake intake = new Intake(this.hardwareMap);
        Fold fold = new Fold(this.hardwareMap);
        Cover cover = new Cover(this.hardwareMap);

        waitForStart();

        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        fold.setDrive();
        cover.close();


        while (opModeIsActive() && !isStopRequested()) {
            fold.setPosition(foldPos);

            intake.setPower(intakePower);

            gamepadEx2.update();
            log.tick();
        }
    }
}
