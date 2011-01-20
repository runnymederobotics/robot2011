package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    
    static final double COUNTS_PER_METRE = 900.0;
    static final double ELEVATOR_SPEED = 1.0;

    static final int OPERATOR_GROUND_STATE = 1;
    static final int OPERATOR_FEED_STATE = 2;
    static final int OPERATOR_POSONE_STATE = 3;
    static final int OPERATOR_POSTWO_STATE = 4;
    static final int OPERATOR_POSTHREE_STATE = 5;
    static final int OPERATOR_POSFOUR_STATE = 6;
    static final int OPERATOR_POSFIVE_STATE = 7;
    static final int OPERATOR_POSSIX_STATE = 8;
    
    boolean haveEncoders = true;
    
    Joystick stickRight = new Joystick(1);
    Joystick stickLeft = new Joystick(2);

    Joystick stickOperator = new Joystick(3);

    Victor vicRight = new Victor(1);
    Victor vicLeft = new Victor(2);
    Victor vicElevator = new Victor(3);

    PIDEncoder encRight = new PIDEncoder(1, 2);
    PIDEncoder encLeft = new PIDEncoder(3, 4);

    Encoder encElevator = new Encoder(5, 6);

    PIDController pidRight = new PIDController(0.0, 0.00003, 0.0, encRight, vicRight, 0.005);
    PIDController pidLeft = new PIDController(0.0, -0.00003, 0.0, encLeft, vicLeft, 0.005);

    class ElevatorState {
        static final int ground = 0;
        static final int feed = 1;
        static final int posOne = 2;
        static final int posTwo = 3;
        static final int posThree = 4;
        static final int posFour = 5;
        static final int posFive = 6;
        static final int posSix = 7;
    }

    int curElevatorState = ElevatorState.ground;

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

    }

    //Eleveator state machine
    public void elevatorStateMachine(double setPoint)
    {
        //Below setPoint, move up
        if(encElevator.get() < setPoint)
        {
            vicElevator.set(ELEVATOR_SPEED);
        }
        //Above setPoint, move down
        else if(encElevator.get() > setPoint)
        {
            vicElevator.set(-ELEVATOR_SPEED);
        }
        else
        {
            vicElevator.set(0.0);
        }
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        double setPoint = 0.0;

        setPoint = stickOperator.getRawButton(OPERATOR_GROUND_STATE) ? ElevatorState.ground : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_FEED_STATE) ? ElevatorState.feed : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSONE_STATE) ? ElevatorState.posOne : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSTWO_STATE) ? ElevatorState.posTwo : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSTHREE_STATE) ? ElevatorState.posThree : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSFOUR_STATE) ? ElevatorState.posFour : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSFIVE_STATE) ? ElevatorState.posFive : setPoint;
        setPoint = stickOperator.getRawButton(OPERATOR_POSSIX_STATE) ? ElevatorState.posSix : setPoint;

        elevatorStateMachine(setPoint);

        pidLeft.setSetpoint(stickLeft.getAxis(Joystick.AxisType.kY));
        pidRight.setSetpoint(stickRight.getAxis(Joystick.AxisType.kY));
    }

}
