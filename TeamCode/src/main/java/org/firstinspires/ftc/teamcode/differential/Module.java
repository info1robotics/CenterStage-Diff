package org.firstinspires.ftc.teamcode.differential;

public class Module<E> {
    E extendo, lift, hang;

    public Module(E extendo, E lift, E hang) {
        this.extendo = extendo;
        this.lift = lift;
        this.hang = hang;
    }

    public void setExtendo(E extendo) {
        this.extendo = extendo;
    }

    public void setLift(E lift) {
        this.lift = lift;
    }

    public void setHang(E hang) {
        this.hang = hang;
    }

    public E getExtendo() {
        return extendo;
    }

    public E getLift() {
        return lift;
    }

    public E getHang() {
        return hang;
    }
}
