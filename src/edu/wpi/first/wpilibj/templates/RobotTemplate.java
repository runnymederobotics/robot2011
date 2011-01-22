package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    
    static final double COUNTS_PER_METRE = 900.0;
    static final double ELEVATOR_SPEED = 1.0;

    class Driver {
        static final int TRANS_TOGGLE = 1;
    }

    class Operator {
        static final int ELEVATOR_STATE_GROUND = 1;
        static final int ELEVATOR_STATE_FEED = 2;
        static final int ELEVATOR_STATE_ONE = 3;
        static final int ELEVATOR_STATE_TWO = 4;
        static final int ELEVATOR_STATE_THREE = 5;
        static final int ELEVATOR_STATE_FOUR = 6;
        static final int ELEVATOR_STATE_FIVE = 7;
        static final int ELEVATOR_STATE_SIX = 8;
        static final int GRIPPER_TOGGLE = 9;
        static final int ELBOW_TOGGLE = 10;
    }
    
    
    boolean haveEncoders = true;
    
    Joystick stickDriver = new Joystick(1);
    Joystick stickOperator = new Joystick(2);

    Compressor compressor = new Compressor(10, 1);

    Relay transmissionShift = new Relay(2);
    Relay gripper = new Relay(3);
    Relay elbow = new Relay(4);

    Jaguar jagLeftOne = new Jaguar(1);
    Jaguar jagLeftTwo = new Jaguar(2);
    Jaguar jagRightOne = new Jaguar(10);
    Jaguar jagRightTwo = new Jaguar(4);

    Victor vicElevator = new Victor(5);

    PIDEncoder encRight = new PIDEncoder(1, 2);
    PIDEncoder encLeft = new PIDEncoder(3, 4);

    //Encoder encElevator = new Encoder(5, 6);

    //PIDController pidRight = new PIDController(0.0, 0.00003, 0.0, encRight, jagRightOne, 0.005);
    //PIDController pidLeft = new PIDController(0.0, -0.00003, 0.0, encLeft, jagLeftOne, 0.005);

    //Toggle variables for the transmission shifter button
    BooleanHolder transReleased = new BooleanHolder();
    BooleanHolder transDirection = new BooleanHolder(true);

    //Toggle variables for the gripper button
    BooleanHolder gripperReleased = new BooleanHolder();
    BooleanHolder gripperDirection = new BooleanHolder(true);

    //Toggle variables for the elbow button
    BooleanHolder elbowReleased = new BooleanHolder();
    BooleanHolder elbowDirection = new BooleanHolder(true);

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

    //A boolean object, who's value can be changed from inside a function
    public class BooleanHolder {
        //Value of object
        boolean value;

        //Defaults to false
        public BooleanHolder() {
            value = false;
        }

        //Overloaded constructor to take any value of boolean
        public BooleanHolder(boolean val) {
            value = val;
        }

        //Get boolean
        public final boolean get() {
            return value;
        }

        //Set boolean
        public void set(boolean val) {
            value = val;
        }
    }

    //Runs when the robot is turned
    public void robotInit() {
        //Start both of the drive encoders
        encRight.encoder.start();
        encLeft.encoder.start();

        /*pidRight.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidRight.setOutputRange(-1, 1);

        pidLeft.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidLeft.setOutputRange(-1, 1);*/

        //Start the compressor
        compressor.start();
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
        System.out.println("renc: " + encRight.pidGet() + " lenc: " + encLeft.pidGet());
        System.out.println("trans: " + transDirection.get() + " gripper: " + gripperDirection.get() + " elbow: " + elbowDirection.get());
    }

    //Elevator state machine
    /*public void elevatorStateMachine(double setPoint)
    {
        //Below setPoint, move up
        if(encElevator.get() < setPoint) {
            vicElevator.set(ELEVATOR_SPEED);
        }
        //Above setPoint, move down
        else if(encElevator.get() > setPoint) {
            vicElevator.set(-ELEVATOR_SPEED);
        }
        else {
            vicElevator.set(0.0);
        }
    }*/

    //A function to provide toggling capabilities for buttons on controllers
    public void toggle(BooleanHolder value, boolean buttonDown, BooleanHolder buttonReleased) {
        //If the button is pressed and it was released
        if(buttonDown && buttonReleased.get()) {
            //It is no longer released
            buttonReleased.set(false);

            //Toggle the value
            value.set(!value.get());
        }
        //If the button is released
        else if(!buttonDown) {
            //Set the button to released
            buttonReleased.set(true);
        }
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        /*double setPoint = 0.0;

        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_GROUND) ? ElevatorState.ground : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FEED) ? ElevatorState.feed : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_ONE) ? ElevatorState.posOne : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_TWO) ? ElevatorState.posTwo : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_THREE) ? ElevatorState.posThree : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FOUR) ? ElevatorState.posFour : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FIVE) ? ElevatorState.posFive : setPoint;
        setPoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_SIX) ? ElevatorState.posSix : setPoint;

        elevatorStateMachine(setPoint);*/

        //Drive the elevator based on the y axis of the operator joystick
        vicElevator.set(stickOperator.getAxis(Joystick.AxisType.kY));

        //pidLeft.setSetpoint(stickLeft.getAxis(Joystick.AxisType.kY));
        //pidRight.setSetpoint(stickRight.getAxis(Joystick.AxisType.kY));

        //Drive left victors based on the left axis of the joystick
        jagLeftOne.set(stickDriver.getRawAxis(2));
        jagLeftTwo.set(stickDriver.getRawAxis(2));

        //Drive right victors based on the right axis of the joystick
        jagRightOne.set(-stickDriver.getRawAxis(4));
        jagRightTwo.set(-stickDriver.getRawAxis(4));

        //Add a toggle on the transmission shifter button
        toggle(transDirection, stickDriver.getRawButton(Driver.TRANS_TOGGLE), transReleased);

        //Add a toggle on the gripper button
        toggle(gripperDirection, stickOperator.getRawButton(Operator.GRIPPER_TOGGLE), gripperReleased);

        //Add a toggle on the elbow button
        toggle(elbowDirection, stickOperator.getRawButton(Operator.ELBOW_TOGGLE), elbowReleased);

        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Set the gripper to open or closed based on the state of the toggle
        gripper.set(gripperDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Set the elbow to open or closed based on the state of the toggle
        elbow.set(transDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);
    }
}
