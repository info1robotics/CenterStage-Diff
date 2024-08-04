package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.AutoStartPos;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
public class AutoExtento extends AutoBase {
    @Override
    public void onInit() {
        startPos = AutoStartPos.BLUE_LEFT;
        super.onInit();
    }

    @Override
    public void onStartTick() {
        diffy.userTargetPosition.setExtendo(10000);
    }


}
