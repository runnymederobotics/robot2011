package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {

    //Encoder counts per metre travelled by the robot
    static final double COUNTS_PER_METRE = 900.0;

    //Speed to set the elevator motor to
    static final double ELEVATOR_SPEED = 1.0;

    //Driver joystick
    class Driver {
        static final int TRANS_TOGGLE = 1;
        static final int ARCADE_TOGGLE = 2;
    }

    //Operator joystick
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

    //Provides drive functions (arcade and tank drive)
    RobotDrive robotDrive = new RobotDrive(jagLeftOne, jagLeftTwo, jagRightOne, jagRightTwo);

    //Encoder encElevator = new Encoder(5, 6);

    //PID
    PIDController pidRight = new PIDController(0.0, 0.00003, 0.0, encRight, jagRightOne, 0.005);
    PIDController pidLeft = new PIDController(0.0, -0.00003, 0.0, encLeft, jagLeftOne, 0.005);

    //Toggle for the transmission shifter button
    //Default is open -- low gear
    Toggle transToggle = new Toggle(true);

    //Toggle for the gripper button
    //Default is closed -- gripper is closed
    Toggle gripperToggle = new Toggle(false);

    //Toggle for the elbow button
    //Default is closed -- elbow is up
    Toggle elbowToggle = new Toggle(false);

    //Toggle for arcade/tank drive
    //Default is tank drive
    Toggle arcadeToggle = new Toggle(false);

    //Enumeration of setpoints for different heights of the elevator
    class ElevatorSetpoint {
        static final int ground = 0;
        static final int feed = 1;
        static final int posOne = 2;
        static final int posTwo = 3;
        static final int posThree = 4;
        static final int posFour = 5;
        static final int posFive = 6;
        static final int posSix = 7;
    }

    //The elevator setpoint, determined by which button on the operator joystick is pressed
    int elevatorSetpoint = ElevatorSetpoint.ground;

    //Runs when the robot is turned
    public void robotInit() {
        //Start both of the drive encoders
        encRight.start();
        encLeft.start();

        //Input/output range for right encoder/motors
        pidRight.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidRight.setOutputRange(-1, 1);

        //Input/output range for left encoder/motors
        pidLeft.setInputRange(-COUNTS_PER_METRE, COUNTS_PER_METRE);
        pidLeft.setOutputRange(-1, 1);

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
        System.out.println("trans: " + transToggle.get() + " gripper: " + gripperToggle.get() + " elbow: " + elbowToggle.get());
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

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        /*elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_GROUND) ? ElevatorSetpoint.ground : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FEED) ? ElevatorSetpoint.feed : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_ONE) ? ElevatorSetpoint.posOne : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_TWO) ? ElevatorSetpoint.posTwo : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_THREE) ? ElevatorSetpoint.posThree : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FOUR) ? ElevatorSetpoint.posFour : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FIVE) ? ElevatorSetpoint.posFive : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_SIX) ? ElevatorSetpoint.posSix : elevatorSetpoint;

        elevatorStateMachine(setPoint);*/

        //Drive the elevator based on the y axis of the operator joystick
        vicElevator.set(stickOperator.getAxis(Joystick.AxisType.kY));

        //Update the toggle on the transmission shifter button
        transToggle.update(stickDriver.getRawButton(Driver.TRANS_TOGGLE));
        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the gripper button
        gripperToggle.update(stickOperator.getRawButton(Operator.GRIPPER_TOGGLE));
        //Set the gripper to open or closed based on the state of the toggle
        gripper.set(gripperToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the elbow button
        elbowToggle.update(stickOperator.getRawButton(Operator.ELBOW_TOGGLE));
        //Set the elbow to open or closed based on the state of the toggle
        elbow.set(elbowToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Add a toggle on the arcade/tank drive button
        arcadeToggle.update(stickDriver.getRawButton(Driver.ARCADE_TOGGLE));
        //Drive arcade or tank based on the state of the toggle
        if(arcadeToggle.get()) {
            //Move axis is 2 on stickDriver (first y-axis) and rotate axis is 3 on stickDriver (second x-axis)
            robotDrive.arcadeDrive(stickDriver, 2, stickDriver, 3);
        }
        else if (!arcadeToggle.get()) {
            //Left motors controlled by axis 2 on stickDriver (first y axis) and right motors controlled by axis 4 on stickDriver (second y-axis)
            robotDrive.tankDrive(stickDriver.getRawAxis(2), stickDriver.getRawAxis(4));
        }
    }
}
