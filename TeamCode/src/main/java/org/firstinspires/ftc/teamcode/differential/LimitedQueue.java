package org.firstinspires.ftc.teamcode.differential;

import java.util.ArrayList;
import java.util.LinkedList;

public class LimitedQueue<E> extends ArrayList<E> {

    private int limit;

    public LimitedQueue(int limit) {
        this.limit = limit;
    }

    @Override
    public boolean add(E o) {
        boolean added = super.add(o);
        while (added && size() > limit) {
            super.remove(0);
        }
        return added;
    }
}