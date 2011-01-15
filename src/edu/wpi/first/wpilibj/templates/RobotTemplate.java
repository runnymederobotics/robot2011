package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    
    static final double COUNTS_PER_METRE = 900.0;
    
    boolean haveEncoders = true;
    
    Joystick stickRight = new Joystick(1);
    Joystick stickLeft = new Joystick(2);

    Victor vicRight = new Victor(1);
    Victor vicLeft = new Victor(2);

    PIDEncoder encRight = new PIDEncoder(1, 2);
    PIDEncoder encLeft = new PIDEncoder(3, 4);

    PIDController pidRight = new PIDController(0.0, 0.00003, 0.0, encRight, vicRight, 0.005);
    PIDController pidLeft = new PIDController(0.0, -0.00003, 0.0, encLeft, vicLeft, 0.005);

    //Runs when the robot is turned
    public void robotInit() {
        pidRight.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidRight.setOutputRange(-1, 1);

        pidLeft.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidLeft.setOutputRange(-1, 1);
    }

    //Runs at the beginning of autonomous period
    public void autonomousInit() {

    }

    //Runs periodically during autonomous period
    public void autonomousPeriodic() {

    }

    //Runs continuously during autonomous period
    public void autonomousContinuous() {

    }

    //Runs at the beginning of teleoperated period
    public void teleopInit() {

    }

    //Runs periodically during teleoperated period
    public void teleopPeriodic() {
        System.out.println("rX: " + stickRight.getAxis(Joystick.AxisType.kX) + " | rY: " + stickRight.getAxis(Joystick.AxisType.kY));
        System.out.println("lX: " + stickLeft.getAxis(Joystick.AxisType.kX) + " | lY: " + stickLeft.getAxis(Joystick.AxisType.kY));
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        //pidRight.setSetpoint(COUNTS_PER_METRE);
    }

}
