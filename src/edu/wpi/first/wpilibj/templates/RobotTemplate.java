package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    //Encoder rate at max speed in slow gear
    static final double SLOW_MAX_ENCODER_RATE = 750.0;
    //Encoder rate at max speed in fast gear
    static final double FAST_MAX_ENCODER_RATE = 1700.0;
    //Speed to set the elevator motor to
    static final double ELEVATOR_SPEED = 0.4;
    //Max drive motor speed
    static final double MAX_SPEED = 1.0;
    //Starting encoder counts
    static final double ELEVATOR_BASE = 900;
    //Number of elevator encoder counts
    static final int MAX_ELEVATOR_COUNTS = 2800;

    //Driver joystick
    class Driver {
        //Buttons
        static final int TRANS_TOGGLE = 8;
        static final int ARCADE_TOGGLE = 2;

        //Axis
        static final int X_AXIS_LEFT = 1;
        static final int Y_AXIS_LEFT = 2;
        static final int X_AXIS_RIGHT = 3;
        static final int Y_AXIS_RIGHT = 4;
    }

    //Operator joystick
    class Operator {
        //Buttons
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
    PIDController pidLeft = new PIDController(0.0, 0.0005, 0.0, encLeft, jagLeft, 0.005);
    PIDController pidRight = new PIDController(0.0, 0.0005, 0.0, encRight, jagRight, 0.005);
    PIDController pidElevator = new PIDController(0.0, 0.0005, 0.0, encElevator, vicElevator, 1);

    //Toggle for manual elevator control
    //Default is buttons
    Toggle manualElevatorToggle = new Toggle(true);

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

    //Enumeration of setpoints for different heights of the elevator
    class ElevatorSetpoint {
        static final double ground = ELEVATOR_BASE;
        static final double posOne = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS * 1.0 / 6.0;
        static final double posTwo = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS * 2.0 / 6.0;
        static final double posThree = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS * 3.0 / 6.0;
        static final double posFour = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS * 4.0 / 6.0;
        static final double posFive = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS * 5.0 / 6.0;
        static final double posSix = ELEVATOR_BASE + MAX_ELEVATOR_COUNTS;
        static final double feed = ELEVATOR_BASE;
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
        pidLeft.setInputRange(-SLOW_MAX_ENCODER_RATE, SLOW_MAX_ENCODER_RATE);
        pidLeft.setOutputRange(-MAX_SPEED, MAX_SPEED);

        //Input/output range for right encoder/motors
        pidRight.setInputRange(-SLOW_MAX_ENCODER_RATE, SLOW_MAX_ENCODER_RATE);
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
        //Current time
        final double curPrintTime = Timer.getFPGATimestamp();
        //If it has been more than half a second
        if(curPrintTime - lastPrintTime > 0.5) {
            //Print statements
            System.out.println("renc: " + encRight.pidGet() + " lenc: " + encLeft.pidGet() + " elevator: " + encElevator.pidGet());
            System.out.println("rSet: " + pidRight.getSetpoint() + " lSet: " + pidLeft.getSetpoint() + " eSet: " + pidElevator.getSetpoint());
            System.out.println("rPID: " + pidRight.get() + " lPID: " + pidLeft.get() + " ePID: " + pidElevator.get());
            System.out.println("elevatorPID: " + manualElevatorToggle.get());
            System.out.println("elevAxis: " + stickOperator.getAxis(Joystick.AxisType.kY) + "leftAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_LEFT) + " rightAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_RIGHT));
            System.out.println();

            //Update the last print time
            lastPrintTime = curPrintTime;
        }
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        //The elevator setpoint based on the corresponding button
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_GROUND) ? ElevatorSetpoint.ground : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_ONE) ? ElevatorSetpoint.posOne : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_TWO) ? ElevatorSetpoint.posTwo : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_THREE) ? ElevatorSetpoint.posThree : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FOUR) ? ElevatorSetpoint.posFour : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FIVE) ? ElevatorSetpoint.posFive : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_SIX) ? ElevatorSetpoint.posSix : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FEED) ? ElevatorSetpoint.feed : elevatorSetpoint;

        //Actually set our setpoint
        pidElevator.setSetpoint(elevatorSetpoint);

        //Update the toggle on the manual/PID elevator control
        manualElevatorToggle.update(stickOperator.getRawButton(Operator.MANUAL_ELEVATOR_TOGGLE));
        //Manual or PID elevator control
        if(manualElevatorToggle.get()) {
            //Disable PID on the elevator
            pidElevator.disable();
            //Drive the elevator based on the y axis of the operator joystick
            vicElevator.set(-stickOperator.getAxis(Joystick.AxisType.kY));
        } else {
            //Enable PID on the elevator
            pidElevator.enable();
        }

        //Update the toggle on the transmission shifter button
        transToggle.update(stickDriver.getRawButton(Driver.TRANS_TOGGLE));
        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Determine which max encoder rate to use depending on the transmission state
        double maxEncoderRate = transToggle.get() ? FAST_MAX_ENCODER_RATE : SLOW_MAX_ENCODER_RATE;
        pidLeft.setInputRange(-maxEncoderRate, maxEncoderRate);
        pidRight.setInputRange(-maxEncoderRate, maxEncoderRate);

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
            //Disable PID
            pidLeft.disable();
            pidRight.disable();

            //Move axis is first y-axis and rotate axis is second x-axis
            robotDrive.arcadeDrive(stickDriver, Driver.Y_AXIS_LEFT, stickDriver, Driver.X_AXIS_RIGHT);
        }
        else if (!arcadeToggle.get()) {
            //Enable PID
            pidLeft.enable();
            pidRight.enable();

            //Left axis
            double leftAxis = stickDriver.getRawAxis(Driver.Y_AXIS_LEFT);
            //Any value less than 0.2 is set to 0.0 to create a dead zone
            leftAxis = Math.abs(leftAxis) < 0.2 ? 0.0 : leftAxis;

            //Right axis
            double rightAxis = stickDriver.getRawAxis(Driver.Y_AXIS_RIGHT);
            //Any value less than 0.2 is set to 0.0 to create a dead zone
            rightAxis = Math.abs(rightAxis) < 0.2 ? 0.0 : rightAxis;
            
            //Set the setpoint as a percentage of the maximum encoder rate
            pidLeft.setSetpoint(leftAxis * maxEncoderRate);
            pidRight.setSetpoint(-rightAxis * maxEncoderRate);
        }
    }
}
