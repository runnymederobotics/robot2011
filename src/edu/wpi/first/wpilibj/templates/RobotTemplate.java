package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    
    static final double COUNTS_PER_METRE = 900.0;
    static final double ELEVATOR_SPEED = 1.0;

    //Buttons on the driver joystick
    class Driver {
        static final int TRANS_TOGGLE = 1;
        static final int ARCADE_TOGGLE = 2;
    }

    //Buttons on the operator joystick
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

    //Joysticks
    Joystick stickDriver = new Joystick(1);
    Joystick stickOperator = new Joystick(2);

    //Compressor, switch is DI 10, spike is relay 1
    Compressor compressor = new Compressor(10, 1);

    //Relays
    Relay transmissionShift = new Relay(2);
    Relay gripper = new Relay(3);
    Relay elbow = new Relay(4);

    //Jaguars
    Jaguar jagLeftOne = new Jaguar(1);
    Jaguar jagLeftTwo = new Jaguar(2);
    Jaguar jagRightOne = new Jaguar(10);
    Jaguar jagRightTwo = new Jaguar(4);

    //Victors
    Victor vicElevator = new Victor(5);

    //Encoders
    PIDEncoder encRight = new PIDEncoder(1, 2, true);
    PIDEncoder encLeft = new PIDEncoder(3, 4, true);

    RobotDrive robotDrive = new RobotDrive(jagLeftOne, jagLeftTwo, jagRightOne, jagRightTwo);

    //Encoder encElevator = new Encoder(5, 6);

    //PID
    //PIDController pidRight = new PIDController(0.0, 0.00003, 0.0, encRight, jagRightOne, 0.005);
    //PIDController pidLeft = new PIDController(0.0, -0.00003, 0.0, encLeft, jagLeftOne, 0.005);

    //Toggle variables for the transmission shifter button
    BooleanHolder transReleased = new BooleanHolder();
    //True means that it defaults to being open
    BooleanHolder transDirection = new BooleanHolder(true);

    //Toggle variables for the gripper button
    BooleanHolder gripperReleased = new BooleanHolder();
    //False means that it defaults to being closed
    BooleanHolder gripperDirection = new BooleanHolder(false);

    //Toggle variables for the elbow button
    BooleanHolder elbowReleased = new BooleanHolder();
    //False means that it defaults to being closed
    BooleanHolder elbowDirection = new BooleanHolder(false);

    //Toggle variables for arcade/tank drive
    BooleanHolder arcadeReleased = new BooleanHolder();
    //False means that it defaults to being tank drive
    BooleanHolder arcadeDrive = new BooleanHolder(true);

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
        System.out.println("renc: " + encRight.encoder.get() + " lenc: " + encLeft.encoder.get());
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

        //Add a toggle on the transmission shifter button
        toggle(transDirection, stickDriver.getRawButton(Driver.TRANS_TOGGLE), transReleased);
        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the gripper button
        toggle(gripperDirection, stickOperator.getRawButton(Operator.GRIPPER_TOGGLE), gripperReleased);
        //Set the gripper to open or closed based on the state of the toggle
        gripper.set(gripperDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the elbow button
        toggle(elbowDirection, stickOperator.getRawButton(Operator.ELBOW_TOGGLE), elbowReleased);
        //Set the elbow to open or closed based on the state of the toggle
        elbow.set(elbowDirection.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the arcade/tank drive button
        toggle(arcadeDrive, stickDriver.getRawButton(Driver.ARCADE_TOGGLE), arcadeReleased);
        //Drive arcade or tank based on the state of the toggle
        if(arcadeDrive.get()) {
            //Move axis is 2 on stickDriver (first y-axis) and rotate axis is 3 on stickDriver (second x-axis)
            robotDrive.arcadeDrive(stickDriver, 2, stickDriver, 3);
        }
        else {
            //Left motors controlled by axis 2 on stickDriver (first y axis) and right motors controlled by axis 4 on stickDriver (second y-axis)
            robotDrive.tankDrive(stickDriver.getRawAxis(2), stickDriver.getRawAxis(4));
        }
    }
}
