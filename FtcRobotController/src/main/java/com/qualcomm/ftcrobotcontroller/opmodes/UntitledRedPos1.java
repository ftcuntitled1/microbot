package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen on 1/18/2016.
 */
public class UntitledRedPos1 extends PushBotTelemetry{

    //--------------------------------------------------------------------------
    // Program Constants
    //--------------------------------------------------------------------------
    //setting the version for Telemetry Data
    String Version = "1.95";

    // Enter the number of Ticks per single Rotation of Motor (ours is 1440)
    final double EncoderTicksPerRotation = 1440;

    // Calculated gear ratio from motor to wheel
    // Number of Teeth on the Tire Axle Gear divided by the Number of Teeth on the Motor Gear
    final double GearRatio = 2;

    // The Full Diameter of the Wheel Tread in Inches
    // Normally marked on the side of the Tire Tread, but might not be in inches.
    final double WheelDiameter = 4;

    // Calculating the Circumference of the Tire based on the above Variables Provided
    final double WheelCircumference = Math.PI * WheelDiameter;

    // The distance between the left and right tires
    final double AxleWidth = 14.5;

    // Add a buffer to the inside turn radius to prevent freezing
    final double AxleWidthBuffer = 1.0;

    // Encoder Tolerance +/- based on ticks not inches
    // Also helps with the prevention of freezing when comparing TargetEncoderTicks
    final double EncoderTolerance = 4;

    double startTime;
    boolean runTimerStarted;

    // Defining some static positions for the Servo Motors
    final double LEFT_GRIP_OPEN_POSITION = 0.0;
    final double LEFT_GRIP_CLOSED_POSITION = 1.0;

    final double RIGHT_GRIP_OPEN_POSITION = 1.0;
    final double RIGHT_GRIP_CLOSED_POSITION = 0.0;

    //final double LEFT_SWEEP_OPEN_POSITION = 0.0;
    final double LEFT_SWEEP_CLOSED_POSITION = 1.0;

    //final double RIGHT_SWEEP_OPEN_POSITION = 1.0;
    final double RIGHT_SWEEP_CLOSED_POSITION = 0.0;

    // Enumerating the potential list of options or cases for our Switch block.
    enum State
    {
        // list out all of the possible states our robot can be set to
        startup,
        turn1,
        forward1,
        turn2,
        raisearm,
        lowerarm,
        back2,
        drop,
        flippers,
        tankright,
        tankleft,
        openhand,
        closehand,
        waiting,
        done,
    }

    // The custom State, which will track where we are in our program run status
    State botstate;
    // Where to next...
    State NextState;
    // A list of system States.

    // Define driving paths as pairs of relative wheel movements in inches (left,right) plus speed %
    // Note: this is a dummy path, and is NOT likely to actually work with YOUR robot.
    final PathSeg[] mBeaconPath = {
            new PathSeg(  0.0,  3.0, 0.2),  // Left
            new PathSeg( 60.0, 60.0, 0.9),  // Forward
            new PathSeg(  1.0,  0.0, 0.2),  // Left
    };

    final PathSeg[] mMountainPath = {
            new PathSeg(  0.0, -3.0, 0.2),  // Left Rev
            new PathSeg(-30.0,-30.0, 0.9),  // Backup
            new PathSeg(-16.0,  0.0, 0.7),  // Right Rev
            new PathSeg( 10.0, 10.0, 0.3),  // Forward
    };

    final double COUNTS_PER_INCH = 240 ;    // Number of encoder counts per inch of wheel travel.

    final double WHITE_THRESHOLD = 0.5 ;
    final double RANGE_THRESHOLD = 0.5 ;
    final double CLIMBER_RETRACT = 0.1 ;
    final double CLIMBER_DEPLOY  = 1.0 ;

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    // Setting the friendly names for our Motors and Servos
    public DcMotor      mLeftDrive;
    public DcMotor      mRightDrive;
    public DcMotor      mLeftArm;
    public Servo        mLeftGripper;
    public Servo        mRightGripper;
    public Servo        mLeftSweeper;
    public Servo        mRightSweeper;

    private int         mLeftEncoderTarget;
    private int         mRightEncoderTarget;

    //Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();    // Time into round
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    private State       mCurrentState;    // Current State Machine State.
    private PathSeg[]   mCurrentPath;     // Array to hold current path
    private int         mCurrentSeg;      // Index of the current leg in the current path

    double mLeftDrivePower;
    double mRightDrivePower;


    //--------------------------------------------------------------------------
    // Demo Hardware
    //--------------------------------------------------------------------------
    public UntitledRedPos1()
    {
        //nothing to do here
    }

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init() {

        // Initializing all of our Hardware components based on our Robot Configurations
        // that we setup in the robot controller phone
        mLeftDrive       = hardwareMap.dcMotor.get("left_drive");
        mRightDrive      = hardwareMap.dcMotor.get("right_drive");
        // Setting the mRightDrive in reverse as it is on the opposite side of the robot
        // this will reverse the power direction so it matches the mLeftDrive
        mRightDrive.setDirection(DcMotor.Direction.REVERSE);

        mLeftArm         = hardwareMap.dcMotor.get("left_arm");

        mLeftGripper     = hardwareMap.servo.get("left_hand");
        mRightGripper    = hardwareMap.servo.get("right_hand");
        mLeftSweeper     = hardwareMap.servo.get("left_sweeper");
        mRightSweeper    = hardwareMap.servo.get("right_sweeper");

        // Setting and Moving the Servos to a specific start position at startup
        mLeftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        mRightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);

        mLeftSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
        mRightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);

        setDrivePowers(0, 0);    // Ensure motors are off
        resetMotorEncoders();   // Reset Encoders to Zero
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    /**
     * This is our main loop ( or program the is run when you hit the play button
     * on the drivers station phone and will continue looping until you hit stop
     */
    /**
     * This is our main loop ( or program the is run when you hit the play button
     * on the drivers station phone and will continue looping until you hit stop
     */
    public void init_loop() {
        // Keep resetting encoders and show the current values

        // Reset Encoders to Zero
        resetDriveEncoders();

        // Send telemetry data to the driver station.
        telemetry.addData("LMC", " Left Motor Current: " + getLeftPosition());
        telemetry.addData("RMC", "Right Motor Current: " + getRightPosition());
        telemetry.addData("VER", "Version: " + Version);
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start() {
        // Setup Robot devices, set initial state and start game clock
        setDrivePowers(0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();            // Zero game clock
        newState(State.startup);

        // Initializing our first Switch case state using our custom botstate
        // where does our robot start?
        //botstate = State.startup;


    }
    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop()
    {
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());

        switch(mCurrentState) {

            case startup:
                // Check to make sure the Robot is ready
                if (encodersAtZero()) {
                    // Encoders are Zero, so we are ready
                    mCurrentState = State.turn1;
                } else {
                    // Encoders are not Zero, so send that back to the drivers station
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case turn1:
                if(BothMotorEncodersZero()) {
                    singleTurnRobot("right", 45, State.forward1);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case forward1:
                if(BothMotorEncodersZero()) {
                    dualMoveRobot("forwards", 10, State.turn2);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case turn2:
                if(BothMotorEncodersZero()) {
                    singleTurnRobot("left", 135, State.back2);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case back2:
                if(BothMotorEncodersZero()) {
                    dualMoveRobot("backwards", 10, State.raisearm);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case tankright:
                if(BothMotorEncodersZero()) {
                    dualTurnRobot("right", 180, State.tankleft);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case tankleft:
                if(BothMotorEncodersZero()) {
                    dualTurnRobot("left",180,State.closehand);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case raisearm:
                if ( raiseArmForTime(0.1, 10)) {
                    mLeftArm.setPower(0.0);
                    targetTimeReached(1);
                    mCurrentState = State.closehand;
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", "Arm Motor Power: " + mLeftArm.getPower());
                }
                break;

            case closehand:
                if(mLeftArm.getPower() == 0){
                    closeGrippers(State.openhand);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", mLeftGripper.getPosition(),
                            mRightGripper.getPosition()));
                }
                break;

            case openhand:
                if(areGrippersClosed()) {
                    openGrippers(State.done);
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", mLeftGripper.getPosition(),
                            mRightGripper.getPosition()));
                }
                break;

            case lowerarm:
                if ( lowerArmForTime(0.1, 10)) {
                    mLeftArm.setPower(0.0);
                    targetTimeReached(1);
                    mCurrentState = State.waiting;
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", "Arm Motor Power: " + mLeftArm.getPower());
                }
                break;

            case waiting:
                if (mStateTime.time() > 10.0)
                {
                    mCurrentState = State.done;
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", "mStateTime: " + mStateTime.time());
                }
                break;

            case done:
                // stop all functions
                break;

            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }


        // Send telemetry data to the driver station.
        /*
        update_telemetry();
        update_gamepad_telemetry();
        telemetry.addData("110", "--------------------------------");
        telemetry.addData("111", "State: " + botstate);
        telemetry.addData("120", "--------------------------------");
        telemetry.addData("121", "Left Motor");
        telemetry.addData("122", "      *  Target: " + mLeftEncoderTarget);
        telemetry.addData("123", "      * Current: " + getLeftPosition());
        telemetry.addData("124", "      *   Power: " + mLeftDrive.getPower());
        telemetry.addData("125", "      *    Done: " + hasmLeftDriveEncoderReachedTarget(mLeftEncoderTarget));
        telemetry.addData("130", "--------------------------------");
        telemetry.addData("131", "Right Motor");
        telemetry.addData("132", "      *  Target: " + mRightEncoderTarget);
        telemetry.addData("133", "      * Current: " + getRightPosition());
        telemetry.addData("134", "      *   Power: " + mRightDrive.getPower());
        telemetry.addData("135", "      *    Done: " + hasmRightDriveEncoderReachedTarget(mRightEncoderTarget));
        telemetry.addData("140", "--------------------------------");
        telemetry.addData("141", "Arm Motor");
        telemetry.addData("142", "      *   Power: " + mLeftArm.getPower());
        telemetry.addData("150", "--------------------------------");
        telemetry.addData("151", "Grippers");
        telemetry.addData("152", "      *  Left Position: " + mLeftGripper.getPosition());
        telemetry.addData("153", "      * Right Position: " + mRightGripper.getPosition());
        telemetry.addData("160", "--------------------------------");
        telemetry.addData("161", "Sweepers");
        telemetry.addData("162", "      *  Left Position: " + mLeftSweeper.getPosition());
        telemetry.addData("163", "      * Right Position: " + mRightSweeper.getPosition());
        telemetry.addData("990", "--------------------------------");
        telemetry.addData("991", "Version: " + Version);
        */

    } // End of start

    // Ending the Autonomous Program
    @Override
    public void stop() {
        setDrivePowers(0, 0);
    } // End of stop


//--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }


    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    void setEncoderTarget(int leftEncoder, int rightEncoder)
    {
        mLeftDrive.setTargetPosition(mLeftEncoderTarget = leftEncoder);
        mRightDrive.setTargetPosition(mRightEncoderTarget = rightEncoder);
    }

    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder)
    {
        mLeftDrive.setTargetPosition(mLeftEncoderTarget += leftEncoder);
        mRightDrive.setTargetPosition(mRightEncoderTarget += rightEncoder);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        mLeftDrive.setPower(Range.clip(leftPower, -1, 1));
        mRightDrive.setPower(Range.clip(rightPower, -1, 1));
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------
    void synchEncoders()
    {
        //	get and set the encoder targets
        mLeftEncoderTarget = mLeftDrive.getCurrentPosition();
        mRightEncoderTarget = mRightDrive.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (mLeftDrive.getMode() != mode)
            mLeftDrive.setMode(mode);

        if (mRightDrive.getMode() != mode)
            mRightDrive.setMode(mode);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition()
    {
        return mLeftDrive.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return mRightDrive.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    boolean moveComplete()
    {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        return ((Math.abs(getLeftPosition() - mLeftEncoderTarget) < EncoderTolerance) &&
                (Math.abs(getRightPosition() - mRightEncoderTarget) < EncoderTolerance));
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    boolean encodersAtZero()
    {
        return ((Math.abs(getLeftPosition()) < EncoderTolerance) &&
                (Math.abs(getRightPosition()) < EncoderTolerance));
    }

    /*
        Begin the first leg of the path array that is passed in.
        Calls startSeg() to actually load the encoder targets.
     */
    private void startPath(PathSeg[] path)
    {
        mCurrentPath = path;    // Initialize path array
        mCurrentSeg = 0;
        synchEncoders();        // Lock in the current position
        runToPosition();        // Enable RunToPosition mode
        startSeg();             // Execute the current (first) Leg
    }

    /*
        Starts the current leg of the current path.
        Must call startPath() once before calling this
        Each leg adds the new relative movement onto the running encoder totals.
        By not reading and using the actual encoder values, this avoids accumulating errors.
        Increments the leg number after loading the current encoder targets
     */
    private void startSeg()
    {
        int Left;
        int Right;

        if (mCurrentPath != null)
        {
            // Load up the next motion based on the current segemnt.
            Left  = (int)(mCurrentPath[mCurrentSeg].mLeft * COUNTS_PER_INCH);
            Right = (int)(mCurrentPath[mCurrentSeg].mRight * COUNTS_PER_INCH);
            addEncoderTarget(Left, Right);
            setDriveSpeed(mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed);

            mCurrentSeg++;  // Move index to next segment of path
        }
    }

    /*
        Determines if the current path is complete
        As each segment completes, the next segment is started unless there are no more.
        Returns true if the last leg has completed and the robot is stopped.
     */
    private boolean pathComplete()
    {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete())
        {
            // Start next Segement if there is one.
            if (mCurrentSeg < mCurrentPath.length)
            {
                startSeg();
            }
            else  // Otherwise, stop and return done
            {
                mCurrentPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed();
                return true;
            }
        }
        return false;
    }

    /**
     * Custom Methods for our Autonomous Program
     */

    boolean lowerArmForTime(double power, double targetTime)
    {
        mLeftArm.setPower(-power);
        return targetTimeReached(targetTime);
    }

    boolean raiseArmForTime(double power, double targetTime)
    {
        mLeftArm.setPower(power);
        return targetTimeReached(targetTime);
    }

    boolean targetTimeReached(double targetTime)
    {
        if (!runTimerStarted) {
            runTimerStarted = true;
            startTime = getRuntime();
            return false;
        } else {
            boolean result = (getRuntime() - startTime) >= targetTime;
            if (result) {
                runTimerStarted = false;
            }
            return true;
        }
    }


    /**
     * Simple Method for setting the DcMotor to run using the Encoders by name
     */
    public void runUsingEncoders ()
    {
        if (mLeftDrive != null)
        {
            mLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        if (mRightDrive != null)
        {
            mRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    /**
     * Simple Method for resting DcMotor Encoders
     */
    public void resetMotorEncoders ()
    {
        if (mLeftDrive != null)
        {
            mLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
        if (mRightDrive != null)
        {
            mRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    /**
     * Checking to see if the Both Motor Encoders Reached Targets
     */
    boolean haveBothMotorEncodersReachedTargets( double mLeftEncoderTarget, double mRightEncoderTarget)
    {
        boolean BothMotorEncodersReachedTargets = false;
        if (hasmLeftDriveEncoderReachedTarget(mLeftEncoderTarget) &&
                hasmRightDriveEncoderReachedTarget(mRightEncoderTarget))
        {
            BothMotorEncodersReachedTargets = true;
        }
        return BothMotorEncodersReachedTargets;
    }

    /**
     * Checking to see if the Left Motor Encoders Reached Target
     */
    boolean hasmLeftDriveEncoderReachedTarget (double mLeftDriveTicks)
    {
        boolean mLeftDriveEncoderReachedTarget = false;
        if (mLeftDrive != null)
        {
            if (Math.abs(getLeftPosition() - mLeftEncoderTarget) < EncoderTolerance)
            {
                mLeftDriveEncoderReachedTarget = true;
            }
        }
        return mLeftDriveEncoderReachedTarget;
    }

    /**
     * Checking to see if the Right Motor Encoders Reached Target
     */
    boolean hasmRightDriveEncoderReachedTarget (double mRightDriveTicks)
    {
        boolean mRightDriveEncoderReachedTarget = false;
        if (mRightDrive != null)
        {
            if (Math.abs(getRightPosition() - mRightEncoderTarget) < EncoderTolerance)
            {
                mRightDriveEncoderReachedTarget = true;
            }
        }
        return mRightDriveEncoderReachedTarget;
    }

    /**
     * Have BothMotorEncoders been reset to Zero
     */
    boolean BothMotorEncodersZero () {
        boolean BothMotorEncodersZero = false;
        if (ismLeftDriveEncoderZero() && isRigthMotorEncoderZero()) {
            BothMotorEncodersZero = true;
        }
        return BothMotorEncodersZero;
    }
    // Checking to see if the mLeftDriveEncoder is Zero
    boolean ismLeftDriveEncoderZero ()
    {
        boolean mLeftDriveEncoderZero = false;
        if (mLeftDrive.getCurrentPosition() == 0)
        {
            mLeftDriveEncoderZero = true;
        }
        return mLeftDriveEncoderZero;
    }

    // Checking to see if the mRightDriveEncoder is Zero
    boolean isRigthMotorEncoderZero ()
    {
        boolean mRightDriveEncoderZero = false;
        if (mRightDrive.getCurrentPosition () == 0)
        {
            mRightDriveEncoderZero = true;
        }
        return mRightDriveEncoderZero;
    }

    /**
     * Simple Method for setting the DcMotor power by name of Motor
     */
    public void setDrivePowers (double leftPower, double rightPower)

    {
        if (mLeftDrive != null)
        {
            mLeftDrive.setPower(leftPower);
        }
        if (mRightDrive != null)
        {
            mRightDrive.setPower(rightPower);
        }

    }

    /**
     * Simple Method for setting the DcMotor power by name of Motor
     */
    public void setMotorPower (DcMotor Motor, double Power)

    {
        if (Motor != null)
        {
            Motor.setPower(Power);
        }
    }

    /**
     * Simple Method to Close Grippers
     */
    public void closeGrippers(State NextState)
    {
        mLeftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
        mRightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);
        if(areGrippersClosed()) {
            mCurrentState = NextState;
        }
    }

    boolean areGrippersClosed()
    {
        boolean GrippersClosed = false;
        if((mLeftGripper.getPosition() == LEFT_GRIP_CLOSED_POSITION)&&
                (mRightGripper.getPosition() == RIGHT_GRIP_CLOSED_POSITION))
        {
            GrippersClosed = true;
        }
        return GrippersClosed;
    }

    /**
     * Simple Method to Open Grippers
     */
    public void openGrippers(State NextState)
    {
        mLeftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        mRightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);
        if(areGrippersOpen()) {
            mCurrentState = NextState;
        }
    }

    boolean areGrippersOpen()
    {
        boolean GrippersOpen = false;
        if((mLeftGripper.getPosition() == LEFT_GRIP_OPEN_POSITION)&&
                (mRightGripper.getPosition() == RIGHT_GRIP_OPEN_POSITION))
        {
            GrippersOpen = true;
        }
        return GrippersOpen;
    }

    /**
     * This Method will calculate the number of MotorTicksToTurn for the motor to turn
     * based on the given parameter of distanceInches when called
     * param distanceInches (0-999)
     * return - the results in MotorTicks
     */
    public double driveInchesWithEncoder(double distanceInches)
    {
        double WheelRotations = distanceInches / WheelCircumference;
        // returning the MotorTicksToTurn;
        return WheelRotations * EncoderTicksPerRotation * GearRatio;
    }

    /**
     * This Method will return the value for the MotorTicksToTurnOuter or MotorTicksToTurnInner
     * for the motor to turn based on the Gyro Degrees when called adn dynamically change the
     * Inner and Outer based on the WheelLocation
     * param Degrees (0 - 360)
     * param WheelLocation (inner or outer)
     * return - the results in MotorTicks
     */
    public double calculateRobotGyroForDegrees(double Degrees, String WheelLocation)
    {

        double RobotTurnDiameter = 0.0;
        if (WheelLocation.equalsIgnoreCase("inner"))
        {
            // Set to the AxleWidth plus the AxleWidthBoffer, if is the outside wheel turning
            RobotTurnDiameter = AxleWidthBuffer * 2;
        }
        if (WheelLocation.equalsIgnoreCase("outer"))
        {
            // Set to the AxleWidthBuffer only, if is the inner wheel turning
            RobotTurnDiameter = (AxleWidthBuffer + AxleWidth) * 2;
        }
        // Total Circumference in Inches for the Robot Turn Radius of 360
        double RobotTurnCircumferenceInches = RobotTurnDiameter * Math.PI;
        // The number of WheelRotations to drive the full Robot Turn Radius of 360
        double GyroWheelRotations = RobotTurnCircumferenceInches / WheelCircumference;
        // The number of EncoderTicks for the Motor to turn for the full Robot Turn Radius of 360
        double TurnCircumferenceMotorTicks = GyroWheelRotations * EncoderTicksPerRotation * GearRatio;
        // Degrees for how far we want to turn out of the 360, so a straight ratio here
        double TurnRatio = Degrees / 360;
        // Now wrapping it up with the Ratio * TurnCircumferenceTicks to get our partial turn
        // return these results back to the part of the program that called this method
        return TurnRatio * TurnCircumferenceMotorTicks;
    }

    /**
     * Lets make this robot move based on
     * param Distance - (0 - 999 Inches)
     * param Direction - (Forward, Backwards, RTurn, LTurn)
     */
    public void dualMoveRobot(String Direction, double Distance, State NextState) {

        resetMotorEncoders();
        runUsingEncoders();

        mLeftEncoderTarget = (int) driveInchesWithEncoder(0);
        mRightEncoderTarget = (int) driveInchesWithEncoder(Distance);

        mLeftDrive.setTargetPosition(mLeftEncoderTarget);
        mRightDrive.setTargetPosition(mRightEncoderTarget);

        mLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        mRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if (Direction.equalsIgnoreCase("forwards")) {
            //mLeftDrivePower = 1.0f;
            //mRightDrivePower = 1.0f;
            setMotorPower(mLeftDrive, 1.0f);
            setMotorPower(mRightDrive, 1.0f);
        }
        if (Direction.equalsIgnoreCase("backwards")) {
            //mLeftDrivePower = -1.0f;
            //mRightDrivePower = -1.0f;
            setMotorPower(mLeftDrive, -1.0f);
            setMotorPower(mRightDrive, -1.0f);
        }

        if (haveBothMotorEncodersReachedTargets(mLeftEncoderTarget, mRightEncoderTarget)) {
            setMotorPower(mLeftDrive, 0.0f);
            setMotorPower(mRightDrive, 0.0f);
            resetMotorEncoders();
        }
        if (BothMotorEncodersZero()) {
            mCurrentState = NextState;
        }
    }

    public void singleTurnRobot(String Direction, int Degrees, State NextState){

        resetMotorEncoders();
        runUsingEncoders();

        double mLeftEncoderTarget = 0;
        float mLeftDrivePower = 0.0f;

        double mRightEncoderTarget = 0;
        float mRightDrivePower = 0.0f;

        if(Direction.equalsIgnoreCase("right")) {
            mLeftEncoderTarget = calculateRobotGyroForDegrees(Degrees, "inner");
            mLeftDrivePower = 0.07f;
            mRightEncoderTarget = calculateRobotGyroForDegrees(Degrees, "outer");
            mRightDrivePower = 1.00f;
        }
        if(Direction.equalsIgnoreCase("left")){
            mLeftEncoderTarget = calculateRobotGyroForDegrees(Degrees, "outer");
            mLeftDrivePower = 1.00f;
            mRightEncoderTarget = calculateRobotGyroForDegrees(Degrees, "inner");
            mRightDrivePower = 0.07f;
        }
        mLeftDrive.setTargetPosition((int) mLeftEncoderTarget);
        mRightDrive.setTargetPosition((int) mRightEncoderTarget);

        mLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        mRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        setMotorPower(mLeftDrive, mLeftDrivePower);
        setMotorPower(mRightDrive, mRightDrivePower);

        if (haveBothMotorEncodersReachedTargets(mLeftEncoderTarget, mRightEncoderTarget)) {
            setMotorPower(mLeftDrive, 0.0f);
            setMotorPower(mRightDrive, 0.0f);
            resetMotorEncoders();
        }
    }
    public void dualTurnRobot(String Direction, int Degrees, State NextState){

        resetMotorEncoders();
        runUsingEncoders();

        if(Direction.equalsIgnoreCase("left")) {
            mLeftEncoderTarget = (int) (calculateRobotGyroForDegrees(Degrees, "outer")/2);
            mLeftDrivePower = 1.00f;
            mRightEncoderTarget = (int) (calculateRobotGyroForDegrees(Degrees, "outer")/2);
            mRightDrivePower = -1.00f;
        }
        if(Direction.equalsIgnoreCase("right")){
            mLeftEncoderTarget = (int) (calculateRobotGyroForDegrees(Degrees, "outer")/2);
            mLeftDrivePower = -1.00f;
            mRightEncoderTarget = (int) (calculateRobotGyroForDegrees(Degrees, "outer")/2);
            mRightDrivePower = 1.00f;
        }
        mLeftDrive.setTargetPosition(mLeftEncoderTarget);
        mRightDrive.setTargetPosition(mRightEncoderTarget);

        mLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        mRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        setMotorPower(mLeftDrive, mLeftDrivePower);
        setMotorPower(mRightDrive, mRightDrivePower);

        if (haveBothMotorEncodersReachedTargets(mLeftEncoderTarget, mRightEncoderTarget)) {
            setMotorPower(mLeftDrive,  0.0f);
            setMotorPower(mRightDrive, 0.0f);
            resetMotorEncoders();
            if(BothMotorEncodersZero()) {
                mCurrentState = NextState;
            }
        }
    }
}

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
class PathSeg
{
    public double mLeft;
    public double mRight;
    public double mSpeed;

    // Constructor
    public PathSeg(double Left, double Right, double Speed)
    {
        mLeft = Left;
        mRight = Right;
        mSpeed = Speed;
    }
}