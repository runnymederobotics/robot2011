package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

//A storage class to hold the output of a PIDController
public class PIDOutputStorage implements PIDOutput {
    public void pidWrite(double output) {
        value = output;
    }

    public double get() {
        return value;
    }

    double value = 0;
};