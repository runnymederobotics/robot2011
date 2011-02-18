package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class Pneumatic {
    Solenoid singleSolenoid;
    DoubleSolenoid doubleSolenoid;
    Relay relay;

    public Pneumatic(Solenoid s) {
        singleSolenoid = s;
    }

    public Pneumatic(DoubleSolenoid d) {
        doubleSolenoid = d;
    }

    public Pneumatic(Relay r) {
        relay = r;
    }

    public void set(boolean val) {
        if(singleSolenoid != null)
            singleSolenoid.set(val);
        else if(doubleSolenoid != null)
            doubleSolenoid.set(val ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        else if(relay != null)
            relay.set(val ? Relay.Value.kForward : Relay.Value.kReverse);
    }
}
