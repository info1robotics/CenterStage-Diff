package org.firstinspires.ftc.teamcode.common;

import java.util.ArrayList;

public class ActionQueue extends ArrayList<ScheduledRunnable> {
    public void tick() {
        for (int i = 0; i < size(); i++) {
            ScheduledRunnable action = get(i);
            if (action.isDue()) {
                action.run();
                remove(i);
                i--;
            }
        }
    }

    public void clear(String type) {
        for (int i = 0; i < size(); i++) {
            if (get(i).getType().equals(type)) {
                remove(i);
                i--;
            }
        }
    }
}

