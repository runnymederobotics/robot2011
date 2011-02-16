package edu.wpi.first.wpilibj.templates;

public class ButtonPress {
    boolean value = false;
    boolean lastValue = false;

    public void feed(boolean val) {
        if(!lastValue && val)
            value = true;
        else
            value = false;
        lastValue = val;
    }

    public boolean get() {
        return value;
    }
}
