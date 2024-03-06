package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(group = "Debug")
public class DirectionDebug extends LinearOpMode {
    double p(boolean b) {
        return b ? 1 : 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        new Log(this.telemetry);

        Drivetrain drive = new Drivetrain(hardwareMap);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.fl.setPower(p(gamepad1.dpad_up));
            drive.fr.setPower(p(gamepad1.dpad_right));
            drive.br.setPower(p(gamepad1.dpad_down));
            drive.bl.setPower(p(gamepad1.dpad_left));
        }
    }
}
