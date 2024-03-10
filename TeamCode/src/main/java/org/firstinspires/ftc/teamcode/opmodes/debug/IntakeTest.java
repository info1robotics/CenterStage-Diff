package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.Log;
import org.firstinspires.ftc.teamcode.subsystems.Cover;
import org.firstinspires.ftc.teamcode.subsystems.Fold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(group = "Debug")
@Config
public class IntakeTest extends LinearOpMode {
    public static boolean coverClosed = true;
    public static boolean foldInit = true;
    public static double intakePower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log(telemetry);
        Intake intake = new Intake(this.hardwareMap);
        Fold fold = new Fold(this.hardwareMap);
        Cover cover = new Cover(this.hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (coverClosed) cover.setClosed();
            else cover.setOpen();

            if (foldInit) fold.setInit();
            else fold.setDrive();

            intake.setPower(intakePower);

            log.tick();
        }
    }
}
