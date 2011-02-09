package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class RobotTemplate extends IterativeRobot {
    //Encoder rate at max speed in slow gear
    static final double SLOW_MAX_ENCODER_RATE = 750.0;
    //Encoder rate at max speed in fast gear
    static final double FAST_MAX_ENCODER_RATE = 1700.0;
    //Speed to set the elevator motor to
    static final double ELEVATOR_SPEED = 0.8;
    //Max drive motor speed
    static final double MAX_DRIVE_SPEED = 1.0;
    //Encoder counts per metre travelled
    static final double COUNTS_PER_METRE = 500;
    //Starting encoder counts
    static final double ELEVATOR_BASE = 0;
    //Number of elevator encoder counts
    static final int MAX_ELEVATOR_COUNTS = 2800;
    //Number of seconds to wait in teleoperated mode before the minibot is allowed to be deployed
    static final double MINIBOT_RELEASE_TIME = 110.0;
    //Number of seconds after the minibot drops before we send it out horizontal
    static final double MINIBOT_HORIZONTAL_DELAY = 2.0;
    //Tolerance for the gyro pid
    static final double GYRO_TOLERANCE = 1;

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
        static final int ELEVATOR_STATE_ONE = 11;
        static final int ELEVATOR_STATE_TWO = 12;
        static final int ELEVATOR_STATE_THREE = 9;
        static final int ELEVATOR_STATE_FOUR = 10;
        static final int ELEVATOR_STATE_FIVE = 7;
        static final int ELEVATOR_STATE_SIX = 8;
        static final int ELEVATOR_STATE_FEED = 2;
        static final int ELEVATOR_MANUAL_TOGGLE = 5;
        static final int GRIPPER_TOGGLE = 3;
        static final int ELBOW_UP = 6;
        static final int ELBOW_DOWN = 4;
        static final int MINIBOT_RELEASE_ONE = 5;
        static final int MINIBOT_RELEASE_TWO = 6;
    }

    //Driver station
    DriverStation ds = DriverStation.getInstance();

    //Joysticks
    Joystick stickDriver = new Joystick(1);
    Joystick stickOperator = new Joystick(2);

    //Compressor, switch is DI 10, spike is relay 1
    Compressor compressor = new Compressor(10, 1);

    //Relays
    Relay transmissionShift = new Relay(2);
    Relay gripper = new Relay(3);
    Relay elbowOne = new Relay(4);
    Relay elbowTwo = new Relay(5);
    Relay minibotVertical = new Relay(6);
    Relay minibotHorizontal = new Relay(7);

    //A storage class to hold the output of a PIDController
    class PIDOutputStorage implements PIDOutput {
        //pidWrite override
        public void pidWrite(double output) {
            value = output;
        }

        //Get the value
        public double get() {
            return value;
        }

        //The output of the PID
        double value = 0;
    };

    //A storage class to hold the output of a PIDController
    class OutputStorage {
        public void disable() {
        }

        public void set(double val) {
            value = val;
        }

        public void set(double val, byte i) {
            value = val;
        }

        public void pidWrite() {

        }

        //Get the value
        public double get() {
            return value;
        }

        //The output of the PID
        double value = 0;
    };

    //Gyro
    Gyro gyro = new Gyro(1);
    PIDOutputStorage gyroOutput = new PIDOutputStorage();
    PIDController pidGyro = new PIDController(0.0, 0.0005, 0.0, gyro, gyroOutput, 0.005);

    //Jaguars
    Jaguar jagLeft = new Jaguar(1);
    Jaguar jagRight = new Jaguar(2);

    //Stores output from robotDrive
    OutputStorage storageLeft = new OutputStorage();
    OutputStorage storageRight = new OutputStorage();

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

    //Toggle for manual or automated elevator control
    //Default -- automated
    Toggle manualElevatorToggle = new Toggle(false);

    //Toggle for the transmission shifter button
    //Default -- low gear
    Toggle transToggle = new Toggle(false);

    //Toggle for the gripper button
    //Default -- gripper is closed
    Toggle gripperToggle = new Toggle(false);

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
        //Start our encoders
        encRight.start();
        encLeft.start();
        encElevator.start();

        //Start our elevator encoder at 0
        encElevator.reset();

        //Input/output range for left encoder/motors
        pidLeft.setInputRange(-SLOW_MAX_ENCODER_RATE, SLOW_MAX_ENCODER_RATE);
        pidLeft.setOutputRange(-MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        //Input/output range for right encoder/motors
        pidRight.setInputRange(-SLOW_MAX_ENCODER_RATE, SLOW_MAX_ENCODER_RATE);
        pidRight.setOutputRange(-MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        //Input/output range for the gyro PID
        pidGyro.setInputRange(-360.0, 360.0);
        pidGyro.setOutputRange(-0.2, 0.2);

        //Start the compressor
        compressor.start();
    }
    
    //Used in teleopPeriodic to print only once a second
    double lastPrintTime = 0;

    //Print function for our variables
    public void print(String mode) {
        //Current time
        final double curPrintTime = Timer.getFPGATimestamp();
        //If it has been more than half a second
        if(curPrintTime - lastPrintTime > 0.5) {
            //Print statements
            System.out.println("[" + mode + "]");
            System.out.println("DS DI 1: " + ds.getDigitalIn(1));
            System.out.println("renc: " + encRight.encoder.get() + " lenc: " + encLeft.encoder.get() + " elevator: " + encElevator.pidGet());
            System.out.println("rSet: " + pidRight.getSetpoint() + " lSet: " + pidLeft.getSetpoint() + " eSet: " + elevatorSetpoint);
            System.out.println("rPID: " + pidRight.get() + " lPID: " + pidLeft.get());
            System.out.println("manualElevator: " + manualElevatorToggle.get());
            System.out.println("elevAxis: " + stickOperator.getAxis(Joystick.AxisType.kY) + " leftAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_LEFT) + " rightAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_RIGHT));
            System.out.println("Gyro PIDget: " + gyro.pidGet() + " gyro output storage: " + gyroOutput.get());
            System.out.println();

            //Update the last print time
            lastPrintTime = curPrintTime;
        }
    }

    //Runs at the beginning of disabled period
    public void disabledInit() {
        //Disable PIDs
        pidLeft.disable();
        pidRight.disable();
        pidGyro.disable();
    }
    
    //Runs periodically during disabled period
    public void disabledPeriodic() {
        //Call our print function with the current mode
        print("Disabled");
    }

    //Runs continuously during disabled period
    public void disabledContinuous() {
    }
    
        //Enumeration of autonomous modes
    class AutonomousState {
        static final int Driving = 0;
        static final int Turning = 1;
        static final int Hanging = 2;
        static final int Done = 3;
        static final int Sleep = 4;
    }

    //Class that defines the current step in autonomous mode
    class Step {
        //Constructor for Step
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

    Step autoOne[] = {
                        new Step(AutonomousState.Turning, 90),
                        new Step(AutonomousState.Sleep, 5),
                        new Step(AutonomousState.Driving, 1),
                        new Step(AutonomousState.Sleep, 5),
                        new Step(AutonomousState.Turning, -90),
                        new Step(AutonomousState.Sleep, 5),
                        new Step(AutonomousState.Done, 0),
                    };

    //Array to hold steps -- changed depending on which autonomous mode we want
    Step stepList[] = autoOne;
    //Iterates through each step
    int stepIndex;
    //Number of times our setpoint has been reached
    int gyroCounter;

    //Runs at the beginning of autonomous period
    public void autonomousInit() {
        //Minibot defaults to up
        minibotVertical.set(Relay.Value.kReverse);
        minibotHorizontal.set(Relay.Value.kReverse);

        //Default to slow driving mode
        transmissionShift.set(Relay.Value.kReverse);

        //Reset gyro and enable PID on gyro
        pidGyro.enable();
        gyro.reset();

        //Enable PID on wheels
        pidLeft.enable();
        pidRight.enable();

        //Reset encoders
        encLeft.reset();
        encRight.reset();

        //Current step
        stepIndex = 0;

        //We havent reached our setpoint
        gyroCounter = 0;
    }

    //Runs periodically during autonomous period
    public void autonomousPeriodic() {
        //Call our print function with the current mode
        print("Autonomous");
    }

    //Runs continuously during autonomous period
    public void autonomousContinuous() {
        //Our current step in our list of steps
        Step currentStep = stepList[stepIndex];
        //The last step we did
        int lastStepIndex = stepIndex;
        //If we have a step to do
        if(currentStep != null) {
            //Switch the type of step
            switch(currentStep.type) {
                //If we want to drive forward
                case AutonomousState.Driving:
                    //If we have reached our value for this step on the left or right side
                    final boolean leftDone = -encLeft.encoder.get() / COUNTS_PER_METRE >= currentStep.get();
                    final boolean rightDone = encRight.encoder.get() / COUNTS_PER_METRE >= currentStep.get();
                    //Drive each side until we reach the value for each side
                    if(!leftDone)
                        pidLeft.setSetpoint(-0.4 * SLOW_MAX_ENCODER_RATE);
                    if(!rightDone)
                        pidRight.setSetpoint(0.4 * SLOW_MAX_ENCODER_RATE);
                    //If the value is reached
                    if(leftDone && rightDone)
                        ++stepIndex;
                    break;
                //If we want to turn
                case AutonomousState.Turning:
                    //Disable PIDs for smoother turning
                    if(pidLeft.isEnable() || pidRight.isEnable()) {
                        pidLeft.disable();
                        pidRight.disable();
                    }
                    //Set the setpoint for the gyro PID to the step's setpoint
                    pidGyro.setSetpoint(currentStep.get());
                    //Drive the motors with the output from the gyro PID
                    jagLeft.set(-gyroOutput.get());
                    jagRight.set(-gyroOutput.get());
                    //Difference between our position and our setpoint
                    final double delta = currentStep.get() - gyro.pidGet();
                    //If the gyro is below or above the target angle depending on the direction we are turning
                    if(Math.abs(delta) < GYRO_TOLERANCE)
                        ++gyroCounter;
                    if(gyroCounter >= 10)
                        ++stepIndex;
                    break;
                //If we want to use the elevator
                case AutonomousState.Hanging:
                    break;
                //If we are done our autonomous mode
                case AutonomousState.Done:
                    break;
                //Sleep state
                case AutonomousState.Sleep:
                    double time = currentStep.get();
                    while(time > 0)
                    {
                        print("Autonomous");
                        Timer.delay(1);
                        --time;
                        Watchdog.getInstance().feed();
                    }
                    break;
            }
        }
        //If we want to go to the next step
        if(lastStepIndex != stepIndex) {
            //Reset everything
            encLeft.reset();
            encRight.reset();
            gyro.reset();
            //Enable PIDs
            pidLeft.enable();
            pidRight.enable();
            //Stop
            pidLeft.setSetpoint(0.0);
            pidRight.setSetpoint(0.0);
            //Reset gyro counter to 0
            gyroCounter = 0;
        }
    }

    class ElbowState {
        static final int Horizontal = 0;
        static final int Middle = 1;
        static final int Vertical = 2;
    }
    
    class ButtonPress {
        boolean value = false;
        boolean lastValue = false;

        public void feed(boolean val) {
            if(!lastValue && val)
                value = true;
            else
                value = false;
            lastValue = val;
        }

        public boolean get() {
            return value;
        }
    }

    //Start time for teleoperated mode
    double teleopStartTime;
    //Start time for when the minibot release is triggered
    double minibotReleaseTime;
    //Releasing minibot
    boolean releaseMinibot;
    //State of elbow
    int elbowState;
    ButtonPress elbowUp = new ButtonPress();
    ButtonPress elbowDown = new ButtonPress();

    //Runs at the beginning of teleoperated period
    public void teleopInit() {
        //Initialize variables
        teleopStartTime = Timer.getFPGATimestamp();
        minibotReleaseTime = 0.0;
        releaseMinibot = false;

        //Minibot defaults to up
        minibotVertical.set(Relay.Value.kReverse);
        minibotHorizontal.set(Relay.Value.kReverse);
    }

    //Runs periodically during teleoperated period
    public void teleopPeriodic() {
        //Call our print function with the current mode
        print("Teleoperated");
    }

    //Runs continuously during teleoperated period
    public void teleopContinuous() {
        //Don't allow the gyro to be more or less than 360 degrees
        if(gyro.pidGet() < -360 || gyro.pidGet() > 360)
            gyro.reset();

        //The elevator setpoint based on the corresponding button
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_GROUND) ? ElevatorSetpoint.ground : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_ONE) ? ElevatorSetpoint.posOne : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_TWO) ? ElevatorSetpoint.posTwo : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_THREE) ? ElevatorSetpoint.posThree : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FOUR) ? ElevatorSetpoint.posFour : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FIVE) ? ElevatorSetpoint.posFive : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_SIX) ? ElevatorSetpoint.posSix : elevatorSetpoint;
        elevatorSetpoint = stickOperator.getRawButton(Operator.ELEVATOR_STATE_FEED) ? ElevatorSetpoint.feed : elevatorSetpoint;

        //Feed the toggle on the manual/automated elevator control
        manualElevatorToggle.feed(stickOperator.getRawButton(Operator.ELEVATOR_MANUAL_TOGGLE));
        //Manual or automated elevator control
        if(manualElevatorToggle.get()) {
            double axis = -stickOperator.getAxis(Joystick.AxisType.kY);
            //If we are below 0 then dont allow the elevator to go down
            if(encElevator.pidGet() <= 0)
                axis = Math.max(0, axis);
            if(encElevator.pidGet() >= MAX_ELEVATOR_COUNTS)
                axis = Math.min(0, axis);
            vicElevator.set(axis);
        } else {
            //Difference between setpoint and our position
            final double error = elevatorSetpoint - encElevator.pidGet();

            //We can be off by 5%
            final double toleranceWhileGoingUp = MAX_ELEVATOR_COUNTS * 0.05;
            final double toleranceWhileGoingDown = -MAX_ELEVATOR_COUNTS * 0.05;

            //Different speeds going up/down
            final double speedWhileGoingUp = 1.0;
            final double speedWhileGoingDown = -0.7;

            //Go up when below setpoint, down when above setpoint
            if(error > 0 && error > toleranceWhileGoingUp)
                vicElevator.set(speedWhileGoingUp);
            else if(error < 0 && error < toleranceWhileGoingDown)
                vicElevator.set(speedWhileGoingDown);
            else
                vicElevator.set(0.0);
        }

        //Feed the toggle on the transmission shifter button
        transToggle.feed(stickDriver.getRawButton(Driver.TRANS_TOGGLE));
        //Set the transmission shifter to open or closed based on the state of the toggle
        transmissionShift.set(transToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Determine the input range to use (max encoder rate) to use depending on the transmission state we are in
        double maxEncoderRate = transToggle.get() ? FAST_MAX_ENCODER_RATE : SLOW_MAX_ENCODER_RATE;
        pidLeft.setInputRange(-maxEncoderRate, maxEncoderRate);
        pidRight.setInputRange(-maxEncoderRate, maxEncoderRate);

        //Feed the toggle on the gripper button
        gripperToggle.feed(stickOperator.getRawButton(Operator.GRIPPER_TOGGLE));
        //Set the gripper to open or closed based on the state of the toggle
        gripper.set(gripperToggle.get() ? Relay.Value.kForward : Relay.Value.kReverse);

        //Feed the buttons
        elbowUp.feed(stickOperator.getRawButton(Operator.ELBOW_UP));
        elbowDown.feed(stickOperator.getRawButton(Operator.ELBOW_DOWN));

        if(elbowUp.get())
            elbowState += elbowState < ElbowState.Vertical ? 1 : 0;
        if(elbowDown.get())
            elbowState -= elbowState > ElbowState.Horizontal ? 1 : 0;
        elbowOne.set(elbowState == ElbowState.Horizontal ? Relay.Value.kForward : Relay.Value.kReverse);
        elbowTwo.set(elbowState <= ElbowState.Middle ? Relay.Value.kForward : Relay.Value.kReverse);

        //Feed the toggle on the arcade/tank drive button
        arcadeToggle.feed(stickDriver.getRawButton(Driver.ARCADE_TOGGLE));
        //Drive arcade or tank based on the state of the toggle
        if(arcadeToggle.get()) {
            //If PID is enabled
            if(pidLeft.isEnable() || pidRight.isEnable()) {
                //Disable PID
                pidLeft.disable();
                pidRight.disable();
            }

            //Move axis is first y-axis and rotate axis is second x-axis
            //Let the robotdrive class calculate arcade drive for us
            robotDrive.arcadeDrive(stickDriver, Driver.Y_AXIS_LEFT, stickDriver, Driver.X_AXIS_RIGHT);

            //Get the left and right values from what arcadedrive set
            double left = jagLeft.get();
            double right = jagRight.get();

            //Stop the motors
            jagLeft.set(0.0);
            jagRight.set(0.0);

            pidLeft.setSetpoint(left);
            pidRight.setSetpoint(right);
        }
        else if (!arcadeToggle.get()) {
            //If PID is disabled
            if(!pidLeft.isEnable() || !pidRight.isEnable()) {
                //Enable PID
                pidLeft.enable();
                pidRight.enable();
            }

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

        //If there are 10 seconds left
        if(Timer.getFPGATimestamp() - teleopStartTime >= MINIBOT_RELEASE_TIME) {
            //If we triggered the release, set the release to true, otherwise just leave it
            //Creates a one-way toggle
            releaseMinibot = stickOperator.getRawButton(Operator.MINIBOT_RELEASE_ONE) && stickOperator.getRawButton(Operator.MINIBOT_RELEASE_TWO) ? true : releaseMinibot;
            //If we want to release
            if(releaseMinibot) {
                //Set the vertical relay to released
                minibotVertical.set(Relay.Value.kForward);
                //If the release time is 0 (we haven't set the release time yet) then set the release time
                //Allows us to set the release time only once
                minibotReleaseTime = minibotReleaseTime == 0.0 ? Timer.getFPGATimestamp() : minibotReleaseTime;
                //If it's been at least 2 seconds since the release was triggered
                if(Timer.getFPGATimestamp() - minibotReleaseTime >= MINIBOT_HORIZONTAL_DELAY) {
                    //Set the horizontal relay to released
                    minibotHorizontal.set(Relay.Value.kForward);
                }
            }
        }
    }
}
