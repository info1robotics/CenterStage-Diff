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
    public static boolean coverClosed = true;
    public static boolean foldInit = true;
    public static double intakePower = 1;


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

            if (gamepadEx2.getButtonDown("a")) {
                cover.toggle();
            }

            if (gamepadEx2.getButtonDown("dpad_up")) {
                fold.setPosition(fold.getPosition() + 0.02);
            } else if (gamepadEx2.getButtonDown("dpad_down")) {
                fold.setPosition(fold.getPosition() - 0.02);
            }

//            if (foldInit) fold.setInit();
//            else fold.setDrive();

            intake.setPower(intakePower);

            gamepadEx2.update();
            log.tick();
        }
    }
}
