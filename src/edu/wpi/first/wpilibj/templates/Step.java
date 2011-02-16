package edu.wpi.first.wpilibj.templates;

//Class that defines the current step in autonomous mode
public class Step {
    //Constructors for Step
    public Step(int Type) {
        type = Type;
    }

    public Step(int Type, double Val) {
        type = Type;
        value = Val;
    }

    //The type of step
    public int type = AutonomousState.Done;

    //Get the value
    public double get() {
        return value;
    }

    //The amount we want to move by in the step (units depend on the type of step)
    double value = 0.0;
}