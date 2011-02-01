package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {

    //Encoder counts per metre travelled by the robot in slow gear
    static final double SLOW_COUNTS_PER_METRE = 750.0;
    //Encoder counts per metre travelled by the robot in fast gear
    static final double FAST_COUNTS_PER_METRE = 1800.0;
    //Speed to set the elevator motor to
    static final double ELEVATOR_SPEED = 0.5;
    //Max drive motor speed
    static final double MAX_SPEED = 1.0;
    //Number of elevator encoder counts
    static final int MAX_ELEVATOR_COUNTS = 2800;

    //Driver joystick
    class Driver {
        static final int TRANS_TOGGLE = 8;//Right trigger
        static final int ARCADE_TOGGLE = 2;
    }

    //Operator joystick
    class Operator {
        static final int ELEVATOR_STATE_GROUND = 1;
        static final int ELEVATOR_STATE_FEED = 2;
        static final int ELEVATOR_STATE_ONE = 11;
        static final int ELEVATOR_STATE_TWO = 12;
        static final int ELEVATOR_STATE_THREE = 9;
        static final int ELEVATOR_STATE_FOUR = 10;
        static final int ELEVATOR_STATE_FIVE = 7;
        static final int ELEVATOR_STATE_SIX = 8;
        static final int GRIPPER_TOGGLE = 3;
        static final int ELBOW_TOGGLE = 4;
        static final int MANUAL_ELEVATOR_TOGGLE = 5;
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
    Jaguar jagLeft = new Jaguar(1);
    Jaguar jagRight = new Jaguar(2);

    //DI 3 doesn't work

    //Victors
    Victor vicElevator = new Victor(4);

    //Encoders
    PIDEncoder encLeft = new PIDEncoder(true, 3, 4, true);
    Encoder encNull = new Encoder(7, 8);
    PIDEncoder encElevator = new PIDEncoder(false, 5, 6);
    PIDEncoder encRight = new PIDEncoder(true, 1, 2, true);

    //Provides drive functions (arcade and tank drive)
    RobotDrive robotDrive = new RobotDrive(jagLeft, jagRight);

    //PIDs
    PIDController pidLeft = new PIDController(0.0, 0.0001, 0.0, encLeft, jagLeft, 0.005);
    PIDController pidRight = new PIDController(0.0, 0.0001, 0.0, encRight, jagRight, 0.005);
    PIDController pidElevator = new PIDController(0.0, 0.0003, 0.0, encElevator, vicElevator, 0.005);

    //Toggle for the transmission shifter button
    //Default is open -- low gear
    Toggle transToggle = new Toggle(false);

    //Toggle for the gripper button
    //Default is closed -- gripper is closed
    Toggle gripperToggle = new Toggle(false);

    //Toggle for the elbow button
    //Default is closed -- elbow is up
    Toggle elbowToggle = new Toggle(false);

    //Toggle for arcade/tank drive
    //Default is tank drive
    Toggle arcadeToggle = new Toggle(false);

    //Toggle for manual elevator control
    //Default is buttons
    Toggle manualElevatorToggle = new Toggle(true);

    //Enumeration of setpoints for different heights of the elevator
    class ElevatorSetpoint {
        static final double ground = 0;
        static final double feed = 0;
        static final double posOne = MAX_ELEVATOR_COUNTS * 1.0 / 6.0;
        static final double posTwo = MAX_ELEVATOR_COUNTS * 2.0 / 6.0;
        static final double posThree = MAX_ELEVATOR_COUNTS * 3.0 / 6.0;
        static final double posFour = MAX_ELEVATOR_COUNTS * 4.0 / 6.0;
        static final double posFive = MAX_ELEVATOR_COUNTS * 5.0 / 6.0;
        static final double posSix = MAX_ELEVATOR_COUNTS;
    }

    //The elevator setpoint, determined by which button on the operator joystick is pressed
    double elevatorSetpoint = ElevatorSetpoint.ground;

    //Runs when the robot is turned
    public void robotInit() {
        //Start both of the drive encoders
        encRight.start();
        encLeft.start();
        encElevator.start();

        //Start our elevator encoder at 0
        encElevator.reset();

        //Input/output range for left encoder/motors
        pidLeft.setInputRange(-SLOW_COUNTS_PER_METRE, SLOW_COUNTS_PER_METRE);
        pidLeft.setOutputRange(-MAX_SPEED, MAX_SPEED);

        //Input/output range for right encoder/motors
        pidRight.setInputRange(-SLOW_COUNTS_PER_METRE, SLOW_COUNTS_PER_METRE);
        pidRight.setOutputRange(-MAX_SPEED, MAX_SPEED);

        //Input/output range for elevator encoder/motor
        pidElevator.setInputRange(0, MAX_ELEVATOR_COUNTS);
        pidElevator.setOutputRange(-ELEVATOR_SPEED, ELEVATOR_SPEED);

        //Start the compressor
        compressor.start();
    }

    //Runs at the beginning of disabled period
    public void disabledInit() {
        pidLeft.disable();
        pidRight.disable();
        pidElevator.disable();
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

    //Used in teleopPeriodic to print only once a second
    double lastPrintTime = 0;

    //Runs periodically during teleoperated period
    public void teleopPeriodic() {
        final double curTime = Timer.getFPGATimestamp();
        if(curTime - lastPrintTime > 0.5) {
            System.out.println("renc: " + encRight.pidGet() + " lenc: " + encLeft.pidGet() + " elevator: " + encElevator.pidGet());
            System.out.println("rSet: " + pidRight.getSetpoint() + " lSet: " + pidLeft.getSetpoint() + " eSet: " + pidElevator.getSetpoint());
            System.out.println("rPID: " + pidRight.get() + "lPID: " + pidLeft.get() + " ePID: " + pidElevator.get());
            System.out.println("elevatorPID: " + manualElevatorToggle.get());
            System.out.println();
            //System.out.println("pidL: " + pidLeft.get() + " pidR: " + pidRight.get());
            //System.out.println("elevator setpoint: " + elevatorSetpoint);
            lastPrintTime = curTime;
        }
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        //The elevator setpoint based on the corresponding button
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_GROUND) ? ElevatorSetpoint.ground : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FEED) ? ElevatorSetpoint.feed : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_ONE) ? ElevatorSetpoint.posOne : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_TWO) ? ElevatorSetpoint.posTwo : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_THREE) ? ElevatorSetpoint.posThree : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FOUR) ? ElevatorSetpoint.posFour : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FIVE) ? ElevatorSetpoint.posFive : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_SIX) ? ElevatorSetpoint.posSix : elevatorSetpoint;

        //Actually set our setpoint
        pidElevator.setSetpoint(elevatorSetpoint);

        //Enable our PIDs
        pidLeft.enable();
        pidRight.enable();
        
        manualElevatorToggle.update(stickOperator.getRawButton(Operator.MANUAL_ELEVATOR_TOGGLE));

        if(manualElevatorToggle.get()) {
            pidElevator.enable();
        } else {
            pidElevator.disable();
            //Drive the elevator based on the y axis of the operator joystick
            vicElevator.set(-stickOperator.getAxis(Joystick.AxisType.kY));
        }

        //Update the toggle on the transmission shifter button
        transToggle.update(stickDriver.getRawButton(Driver.TRANS_TOGGLE));
        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);
        
        double countsPerMetre = transToggle.get() ? FAST_COUNTS_PER_METRE : SLOW_COUNTS_PER_METRE;
        pidLeft.setInputRange(-countsPerMetre, countsPerMetre);
        pidRight.setInputRange(-countsPerMetre, countsPerMetre);

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
            //Left axis
            double leftAxis = stickDriver.getRawAxis(2);
            //0.1 dead zone
            leftAxis = Math.abs(leftAxis) < 0.1 ? 0.0 : leftAxis;

            //Right axis
            double rightAxis = stickDriver.getRawAxis(4);
            //0.1 dead zone
            rightAxis = Math.abs(rightAxis) < 0.1 ? 0.0 : rightAxis;
            
            //Left motors controlled by axis 2 on stickDriver (first y axis) and right motors controlled by axis 4 on stickDriver (second y-axis)
            pidLeft.setSetpoint(leftAxis * countsPerMetre);
            pidRight.setSetpoint(-rightAxis * countsPerMetre);
        }
    }
}
