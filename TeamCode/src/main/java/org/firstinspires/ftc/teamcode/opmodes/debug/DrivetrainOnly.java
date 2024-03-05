package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.opmodes.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(group = "Debug")
public class DrivetrainOnly extends LinearOpMode {
    @Override
    public void runOpMode() {

        new Log(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        Drivetrain drive = new Drivetrain(hardwareMap);
        SampleMecanumDrive rrDrive = new SampleMecanumDrive(this.hardwareMap);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                    telemetry.addData("Y:", -gamepad1.left_stick_y);
                    telemetry.addData("X:", -gamepad1.left_stick_x);
                    telemetry.addData("Yaw:", gamepad1.right_trigger - gamepad1.left_trigger);
                    drive.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger, 1);
                }
                gamepadEx1.update();
            }
        }).start();

        while (opModeIsActive() && !isStopRequested()) {
            gamepadEx2.update();
            telemetry.update();
        }
    }
}
