package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class AutoTestCamera extends AutoBase {
    @Override
    public void onInit() {
        startPos = AutoStartPos.BLUE_LEFT;

        task = serial(
                log("Hello, world!")
        );
    }
}
