package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

//Driver joystick
class Driver {
    //Buttons
    static final int TRANS_TOGGLE = 8;
    static final int ARCADE_TOGGLE = 1;

    //Axes
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

//Enumeration of setpoints for different heights of the elevator
class ElevatorSetpoint {
    static final double ground = 0;
    static final double posOne = 250;
    static final double posTwo = 620;
    static final double posThree = 2000;
    static final double posFour = 2360;
    static final double posFive = 3750;
    static final double posSix = 3900;
    static final double feed = 1865;
}

class ElbowState {
    static final int Horizontal = 0;
    static final int Middle = 1;
    static final int Vertical = 2;
}

//Enumeration of autonomous modes
class AutonomousState {
    static final int Driving = 0;
    static final int Turning = 1;
    static final int Hanging = 2;
    static final int Release = 3;
    static final int Done = 4;
    static final int Sleep = 5;
}

public class RobotTemplate extends IterativeRobot {
    //Practise robot or competition robot
    static final boolean PRACTISE_ROBOT = false;
    //Encoder rate at max speed in slow gear
    static final double SLOW_MAX_ENCODER_RATE = 750.0;
    //Encoder rate at max speed in fast gear
    static final double FAST_MAX_ENCODER_RATE = 1700.0;
    //Speed to set the elevator motor to
    static final double ELEVATOR_SPEED_UP = 1.0;
    static final double ELEVATOR_SPEED_DOWN = -0.7;
    //Max drive motor speed
    static final double MAX_DRIVE_SPEED = 1.0;
    //Encoder counts per metre travelled
    static final double COUNTS_PER_METRE = 500;
    //Number of elevator encoder counts
    static final int MAX_ELEVATOR_COUNTS = 2400;
    //Number of seconds to wait in teleoperated mode before the minibot is allowed to be deployed
    static final double MINIBOT_RELEASE_TIME = 110.0;
    //Number of seconds after the minibot drops before we send it out horizontally
    static final double MINIBOT_HORIZONTAL_DELAY = 0.5;
    //Tolerance for the gyro pid
    static final double GYRO_TOLERANCE = 1.0;
    //Delay between
    static final double AUTONOMOUS_RELEASE_DELAY = 0.5;
    //Print delay
    static final double PRINT_DELAY = 0.5;

    //Driver station
    DriverStation ds = DriverStation.getInstance();

    //Joysticks
    Joystick stickDriver = new Joystick(1);
    Joystick stickOperator = new Joystick(2);

    //Compressor, switch is DI 10, spike is relay 1
    Compressor compressor = new Compressor(10, 1);

    //Solenoids for main robot or practise robot
    Pneumatic transShift;
    Pneumatic elbowTop;
    Pneumatic elbowBottom;
    Pneumatic gripper;
    Pneumatic minibotHorizontal;

    //Vertical is always a double solenoid
    DoubleSolenoid minibotVertical;

    //Gyro
    Gyro gyro = new Gyro(1);
    PIDOutputStorage gyroOutput = new PIDOutputStorage();

    //Jaguars
    Jaguar jagLeft = new Jaguar(1);
    Jaguar jagRight = new Jaguar(2);

    //Stores output from robotDrive
    OutputStorage storageLeft = new OutputStorage();
    OutputStorage storageRight = new OutputStorage();

    //DI 3 doesn't work

    //Victors
    Victor vicElevator = new Victor(3);

    //Encoders
    PIDEncoder encLeft;
    Encoder encNull;
    PIDEncoder encElevator;
    PIDEncoder encRight;

    DigitalInput rightSensor = new DigitalInput(11);
    DigitalInput middleSensor = new DigitalInput(12);
    DigitalInput leftSensor = new DigitalInput(13);

    //Provides drive functions (arcade and tank drive)
    RobotDrive robotDrive = new RobotDrive(storageLeft, storageRight);

    //PIDs
    PIDController pidLeft;
    PIDController pidRight;
    PIDController pidGyro;

    //Toggle for manual or automated elevator control
    Toggle manualElevatorToggle = new Toggle(true); //Automated elevator
    //Toggle for the transmission shifter
    Toggle transToggle = new Toggle(false); //Low gear
    //Toggle for the gripper
    Toggle gripperToggle = new Toggle(false); //Gripper closed
    //Toggle for arcade/tank drive
    Toggle arcadeToggle = new Toggle(false); //Tank drive
    
    //State of elbow
    int elbowState;

    ButtonPress elbowUp = new ButtonPress();
    ButtonPress elbowDown = new ButtonPress();

    //The elevator setpoint, determined by which button on the operator joystick is pressed
    double elevatorSetpoint = ElevatorSetpoint.ground;

    //Runs when the robot is turned
    public void robotInit() {
        if(!PRACTISE_ROBOT) {
            encLeft = new PIDEncoder(true, 3, 4, true);
            encNull = new Encoder(7, 8);
            encElevator = new PIDEncoder(false, 5, 6, true);
            encRight = new PIDEncoder(true, 1, 2, true);
        }
        else {
            encLeft = new PIDEncoder(true, 5, 6, true);
            encNull = new Encoder(3, 4);
            encElevator = new PIDEncoder(false, 7, 8);
            encRight = new PIDEncoder(true, 1, 2, true);
        }

        pidLeft = new PIDController(0.0, 0.00025, 0.0, encLeft, jagLeft, 0.005);
        pidRight = new PIDController(0.0, 0.00025, 0.0, encRight, jagRight, 0.005);
        pidGyro = new PIDController(0.0005, 0.0005, 0.0, gyro, gyroOutput, 0.005);

        //Initialize our pneumatics if we are using the practise robot or the real robot
        if(!PRACTISE_ROBOT) {
            transShift = new Pneumatic(new Solenoid(8));
            elbowTop = new Pneumatic(new Solenoid(3));
            elbowBottom = new Pneumatic(new Solenoid(2));
            gripper = new Pneumatic(new Solenoid(1));
            minibotHorizontal = new Pneumatic(new Solenoid(4));
            minibotVertical = new DoubleSolenoid(6, 7);
        }
        else {
            transShift = new Pneumatic(new Relay(5));
            elbowTop = new Pneumatic(new DoubleSolenoid(3, 4));
            elbowBottom = new Pneumatic(new DoubleSolenoid(5, 6));
            gripper = new Pneumatic(new DoubleSolenoid(1, 2));
            minibotHorizontal = new Pneumatic(new Relay(7));
            minibotVertical = new DoubleSolenoid(7, 8);
        }

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
        pidGyro.setOutputRange(-0.5, 0.5);

        //Start the compressor
        compressor.start();
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

    //List of autonomous steps
    Step stepList[] = null;

    //Iterates through each step
    int stepIndex;

    boolean doNothing;
    boolean heightOne;
    boolean heightTwo;
    boolean heightThree;
    boolean staggeredPeg;
    boolean releaseTube;
    double startPosition;

    //Runs at the beginning of autonomous period
    public void autonomousInit() {
        //Digital/analog inputs
        doNothing = ds.getDigitalIn(1);
        heightOne = ds.getDigitalIn(2);
        heightTwo = ds.getDigitalIn(3);
        heightThree = ds.getDigitalIn(4);
        staggeredPeg = ds.getDigitalIn(5);
        releaseTube = ds.getDigitalIn(6);
        startPosition = ds.getAnalogIn(1);

        //Minibot defaults to up
        minibotHorizontal.set(false);
        minibotVertical.set(DoubleSolenoid.Value.kReverse);

        //Default to slow driving mode
        transShift.set(false);

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

        //Reset the counter for how many times the gyro has reached its setpoint
        gyroCounter = 0;

        setElbow(ElbowState.Vertical);

        if(doNothing) {
            stepList = new Step[] {
                new Step(AutonomousState.Done),
            };
        }
        else {
            stepList = new Step [] {
                new Step(AutonomousState.Driving, 2850),
                new Step(AutonomousState.Release),
                new Step(AutonomousState.Done),
            };
        }

        //Determine the setpoint of the elevator
        elevatorSetpoint = (heightOne && !staggeredPeg) ? ElevatorSetpoint.posOne : elevatorSetpoint;
        elevatorSetpoint = (heightOne && staggeredPeg) ? ElevatorSetpoint.posTwo : elevatorSetpoint;

        elevatorSetpoint = (heightTwo && !staggeredPeg) ? ElevatorSetpoint.posThree : elevatorSetpoint;
        elevatorSetpoint = (heightTwo && staggeredPeg) ? ElevatorSetpoint.posFour : elevatorSetpoint;

        elevatorSetpoint = (heightThree && !staggeredPeg) ? ElevatorSetpoint.posFive : elevatorSetpoint;
        elevatorSetpoint = (heightThree && staggeredPeg) ? ElevatorSetpoint.posSix : elevatorSetpoint;
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
                    final boolean leftDone = -encLeft.encoder.get() >= currentStep.get();
                    final boolean rightDone = encRight.encoder.get() >= currentStep.get();
                    //Drive each side until we reach the value for each side
                    robotDrive.arcadeDrive(0.65, gyroPID(true, 0.0));
                    if(!leftDone)
                        pidLeft.setSetpoint(-storageLeft.get() * SLOW_MAX_ENCODER_RATE);
                    else
                        pidLeft.setSetpoint(0.0);
                    if(!rightDone)
                        pidRight.setSetpoint(-storageRight.get() * SLOW_MAX_ENCODER_RATE);
                    else
                        pidRight.setSetpoint(0.0);

                    //If the value is reached
                    if(elevatorPID() && leftDone && rightDone)
                        ++stepIndex;
                    break;
                //If we want to turn
                case AutonomousState.Turning:
                    //Disable PIDs for smoother turning
                    if(pidLeft.isEnable() || pidRight.isEnable()) {
                        pidLeft.disable();
                        pidRight.disable();
                    }
                    if(false) {
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
                        if(gyroCounter >= 10) {
                            ++stepIndex;
                            pidLeft.enable();
                            pidRight.enable();
                        }
                    } else {
                        gyroPID(false, currentStep.get());
                    }
                    break;
                //To release the tube
                case AutonomousState.Release:
                    if(releaseTube) {
                        setElbow(ElbowState.Middle);
                        Timer.delay(AUTONOMOUS_RELEASE_DELAY);
                        setElbow(ElbowState.Horizontal);
                        gripper.set(true);
                        //Toggle the gripper to be open at the beginning of teleop
                        gripperToggle.set(true);
                    }
                    ++stepIndex;
                    break;
                //If we are done our autonomous mode
                case AutonomousState.Done:
                    pidLeft.disable();
                    pidRight.disable();
                    break;
                //Sleep state
                case AutonomousState.Sleep:
                    double time = currentStep.get();
                    pidLeft.disable();
                    pidRight.disable();
                    while(time > 0) {
                        print("Autonomous");
                        Timer.delay(1);
                        --time;
                        Watchdog.getInstance().feed();
                    }
                    pidLeft.enable();
                    pidLeft.enable();
                    ++stepIndex;
                    break;
                default:
                    ++stepIndex;
                    break;
            }
        }
        //If we want to go to the next step
        if(lastStepIndex != stepIndex) {
            //Reset everything
            encLeft.reset();
            encRight.reset();
            gyro.reset();
            vicElevator.set(0.0);
            //Stop
            pidLeft.setSetpoint(0.0);
            pidRight.setSetpoint(0.0);
            //Reset gyro counter to 0
            gyroCounter = 0;
        }
    }

    //Start time for teleoperated mode
    double teleopStartTime;
    //Start time for when the minibot release is triggered
    double minibotReleaseTime;
    //Releasing minibot
    boolean releaseMinibot;

    //Runs at the beginning of teleoperated period
    public void teleopInit() {
        //Initialize variables
        teleopStartTime = Timer.getFPGATimestamp();
        minibotReleaseTime = 0.0;
        releaseMinibot = false;

        //Minibot defaults to up
        minibotHorizontal.set(false);
        minibotVertical.set(DoubleSolenoid.Value.kReverse);
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
            vicElevator.set(-stickOperator.getAxis(Joystick.AxisType.kY));
        } else {
            elevatorPID();
        }

        //Feed the toggle on the transmission shifter button
        transToggle.feed(stickDriver.getRawButton(Driver.TRANS_TOGGLE));
        //Set the transmission shifter to open or closed based on the state of the toggle
        transShift.set(transToggle.get());

        //Determine the input range to use (max encoder rate) to use depending on the transmission state we are in
        double maxEncoderRate = transToggle.get() ? FAST_MAX_ENCODER_RATE : SLOW_MAX_ENCODER_RATE;
        pidLeft.setInputRange(-maxEncoderRate, maxEncoderRate);
        pidRight.setInputRange(-maxEncoderRate, maxEncoderRate);

        //Feed the toggle on the gripper button
        gripperToggle.feed(stickOperator.getRawButton(Operator.GRIPPER_TOGGLE));
        //Set the gripper to open or closed based on the state of the toggle
        if(elbowState != ElbowState.Vertical)
            gripper.set(gripperToggle.get());

        //Feed the buttons
        elbowUp.feed(stickOperator.getRawButton(Operator.ELBOW_UP));
        elbowDown.feed(stickOperator.getRawButton(Operator.ELBOW_DOWN));

        //Horizontal < Middle < Vertical
        /*if(elbowUp.get())
            elbowState += elbowState < ElbowState.Vertical ? 1 : 0;
        if(elbowDown.get())
            elbowState -= elbowState > ElbowState.Horizontal ? 1 : 0;*/

        double elbowInput = -stickOperator.getAxis(Joystick.AxisType.kThrottle);
        if(elbowInput < -0.5)
            elbowState = ElbowState.Horizontal;
        else if(elbowInput > 0.5)
            elbowState = ElbowState.Vertical;
        else
            elbowState = ElbowState.Middle;
        setElbow(elbowState);

        //Feed the toggle on the arcade/tank drive button
        arcadeToggle.feed(stickDriver.getRawButton(Driver.ARCADE_TOGGLE));

        //Drive arcade or tank based on the state of the toggle
        if(arcadeToggle.get()) {
            //If PID is disabled
            if(!pidLeft.isEnable() || !pidRight.isEnable()) {
                //Enable PID
                pidLeft.enable();
                pidRight.enable();
            }

            //Let the robotdrive class calculate arcade drive for us
            robotDrive.arcadeDrive(stickDriver, Driver.Y_AXIS_LEFT, stickDriver, Driver.X_AXIS_RIGHT);
            pidLeft.setSetpoint(storageLeft.get() * maxEncoderRate);
            pidRight.setSetpoint(storageRight.get() * maxEncoderRate);
        }
        else if(!arcadeToggle.get()) {
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
                minibotVertical.set(DoubleSolenoid.Value.kForward);
                //If the release time is 0 (we haven't set the release time yet) then set the release time
                //Allows us to set the release time only once
                minibotReleaseTime = minibotReleaseTime == 0.0 ? Timer.getFPGATimestamp() : minibotReleaseTime;
                //If it's been at least 2 seconds since the release was triggered
                if(Timer.getFPGATimestamp() - minibotReleaseTime >= MINIBOT_HORIZONTAL_DELAY) {
                    //Set the horizontal relay to released
                    minibotHorizontal.set(true);
                }
            }
        }
    }

    //Returns whether or not the setpoint has been reached
    public boolean elevatorPID() {
        //Difference between setpoint and our position
        final double error = elevatorSetpoint - encElevator.pidGet();

        //We can be off by 5%
        final double toleranceWhileGoingUp = MAX_ELEVATOR_COUNTS * 0.05;
        final double toleranceWhileGoingDown = -MAX_ELEVATOR_COUNTS * 0.05;

        //Go up when below setpoint, down when above setpoint
        if(error > 0 && error > toleranceWhileGoingUp)
            vicElevator.set(ELEVATOR_SPEED_UP);
        else if(error < 0 && error < toleranceWhileGoingDown)
            vicElevator.set(ELEVATOR_SPEED_DOWN);
        else {
            vicElevator.set(0.0);
            return true;
        }
        return false;
    }

    public void setElbow(int state) {
        //Update the elbow state
        elbowState = state;
        //For the elbow pneumatics, closed = true open = false
        //The top elbow is only ever closed in the vertical state
        elbowTop.set(elbowState != ElbowState.Vertical);
        //The bottom elbow is only ever open in the horizontal state
        elbowBottom.set(elbowState == ElbowState.Horizontal);
        //If we are vertical then close the gripper
        if(elbowState == ElbowState.Vertical)
            gripper.set(false);
    }
    
    //Number of times our setpoint has been reached
    int gyroCounter;

    public double gyroPID(boolean returnSpeed, double target) {
        //Use our own calculations to get to the setpoint of the gyro
        final double delta = target - gyro.getAngle();
        //For straight driving in autonomous mode
        if(returnSpeed) {
            if(Math.abs(delta) < GYRO_TOLERANCE)
                ++gyroCounter;
            if(gyroCounter >= 100) {
                gyroCounter = 0;
                return 0.0;
            }
            //The speed is incorporated into straight driving so it has to be low
            final double speed = 0.1;
            return delta > 0 ? -speed :speed;
        }
        //For turning on the spot
        else {
            if(Math.abs(delta) < GYRO_TOLERANCE)
                ++gyroCounter;
            if(gyroCounter >= 100)
                ++stepIndex;
            //We are turning on the spot so the turning speed is high
            final double speed = 0.5;
            jagLeft.set(delta >= 0 ? -speed : speed);
            jagRight.set(delta >= 0 ? -speed : speed);
            return 0.0;
        }
    }
    
    double lastPrintTime = 0;

    //Print function for our variables
    public void print(String mode) {
        //Current time
        final double curPrintTime = Timer.getFPGATimestamp();
        //If it has been more than half a second
        if(curPrintTime - lastPrintTime > PRINT_DELAY) {
            //Make a bunch of newlines to clear the screen to only show the current output
            System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            //Print statements
            System.out.println("[" + mode + "]");
            System.out.println("gripperToggle: " + gripperToggle.get());
            System.out.println("renc count: " + encRight.encoder.get() + " lenc count: " + encLeft.encoder.get() + " elevator counts: " + encElevator.pidGet());
            System.out.println("rencRate: " + encRight.pidGet() + " lencRate: " + encLeft.pidGet());
            System.out.println("rSet: " + pidRight.getSetpoint() + " lSet: " + pidLeft.getSetpoint() + " eSet: " + elevatorSetpoint);
            System.out.println("rPID: " + pidRight.get() + " lPID: " + pidLeft.get());
            System.out.println("manualElevator: " + manualElevatorToggle.get());
            System.out.println("elevAxis: " + stickOperator.getAxis(Joystick.AxisType.kY) + " leftAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_LEFT) + " rightAxis: " + stickDriver.getRawAxis(Driver.Y_AXIS_RIGHT));
            System.out.println("Gyro PIDget: " + gyro.pidGet() + " gyro output storage: " + gyroOutput.get());
            System.out.println("jagLeft: " + jagLeft.get() + " jagRight: " + jagRight.get());
            System.out.println("elbow input: " + stickOperator.getThrottle() + "elbowState: " + elbowState);
            //System.out.println("Raven gyro min: " + gyro.min + " max: " + gyro.max + " deadzone: " + gyro.deadzone + " center: " + gyro.center);
            System.out.println("rightSensor: " + rightSensor.get() + " middleSensor: " + middleSensor.get() + " leftSensor: " + leftSensor.get());
            //Update the last print time
            lastPrintTime = curPrintTime;
        }
    }
}
