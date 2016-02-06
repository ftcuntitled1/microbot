package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen on 1/18/2016.
 */
public class UntitledManualGyro extends PushBotTelemetry{

    //setting the version for Telemetry Data
    String Version = "1.45";

    // Enter the number of Ticks per single Rotation of Motor (ours is 1440)
    int EncoderTicksPerRotation = 1440;

    // Calculated gear ratio from motor to wheel
    // Number of Teeth on the Tire Axle Gear divided by the Number of Teeth on the Motor Gear
    int GearRatio = 2;

    // The Full Diameter of the Wheel Tread in Inches
    // Normally marked on the side of the Tire Tread, but might not be in inches.
    int WheelDiameter = 4;

    // Calculating the Circumference of the Tire based on the above Variables Provided
    double WheelCircumference = Math.PI * WheelDiameter;

    // The distance between the left and right tires
    double AxleWidth = 14.5;

    // Add a buffer to the inside turn radius to prevent freezing
    double AxleWidthBuffer = 1.0;

    // Encoder Tolerance +/- based on ticks not inches
    // Also helps with the prevention of freezing when comparing TargetEncoderTicks
    int EncoderTolerance = 4;

    // Test Variables to see if Motors have reached the TargetEncoderTicks
    boolean leftMotorFinished = false;
    boolean rightMotorFinished = false;

    double startTime;
    boolean runTimerStarted;


    // Defining some static positions for the Servo Motors
    double LEFT_GRIP_OPEN_POSITION = 0.0;
    double LEFT_GRIP_CLOSED_POSITION = 1.0;

    double RIGHT_GRIP_OPEN_POSITION = 1.0;
    double RIGHT_GRIP_CLOSED_POSITION = 0.0;

    double LEFT_SWEEP_OPEN_POSITION = 0.0;
    double LEFT_SWEEP_CLOSED_POSITION = 1.0;

    double RIGHT_SWEEP_OPEN_POSITION = 1.0;
    double RIGHT_SWEEP_CLOSED_POSITION = 0.0;

    // Setting the friendly names for our Motors and Servos
    DcMotor leftArm;

    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo leftGripper;
    Servo rightGripper;

    Servo leftSweeper;
    Servo rightSweeper;

    ElapsedTime time;

    double leftMotorTarget = 0;
    double rightMotorTarget = 0;

    // Timers to keep track of time for the robot programming run time
    double sweeperTime = 1.0;
    double dropTime = 1.0;
    double liftTime = 1.0;

    // Enumerating the potential list of options or cases for our Switch block.
    enum State
    {
        // list out all of the possible states our robot can be set to
        startup,
        turn1,
        turn1_wait,
        forward1,
        forward1_wait,
        turn2,
        turn2_wait,
        lift,
        lift_wait,
        back2,
        back2_wait,
        drop,
        drop_wait,
        flippers,
        flippers_wait,
        done
    }


    // The custom State, which will track where we are in our program run status
    State botstate;

    // Where to next...
    State NextState;


    @Override
    public void init() {

        // Starting up the ElapsedTime to track our progress and as a failsafe
        // if need to end a particular botstate
        time = new ElapsedTime();

        // Initializing our first Switch case state using our custom botstate
        // where does our robot start?
        botstate = State.startup;

        // Initializing all of our Hardware components based on our Robot Configurations
        // that we setup in the robot controller phone
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftArm = hardwareMap.dcMotor.get("left_arm");

        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");

        leftSweeper = hardwareMap.servo.get("left_sweeper");
        rightSweeper = hardwareMap.servo.get("right_sweeper");

        // Setting the rightMotor in reverse as it is on the opposite side of the robot
        // this will reverse the power direction so it matches the leftMotor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setting and Moving the Servos to a specific start position at startup
        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);

        leftSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
        rightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);

    }

    /**
     * This is our main loop ( or program the is run when you hit the play button
     * on the drivers station phone and will continue looping until you hit stop
     */
    public void loop() {

        switch(botstate) {

            case startup:
                // cleaning up any historical values of the old run data
                resetMotorEncoders();
                leftMotorTarget = 0;
                rightMotorTarget = 0;
                leftMotor.setTargetPosition(0);
                rightMotor.setTargetPosition(0);
                setMotorPower(leftMotor, 0.0f);
                setMotorPower(rightMotor, 0.0f);
                botstate = State.turn1;
                break;

            case turn1:
                singleTurnRobot("Right", 360, State.drop);
                break;

            case turn1_wait:
                // turn left 45 degrees
                runUsingEncoders();

                leftMotorTarget = calculateRobotGyroForDegrees(360, "inner");
                rightMotorTarget = calculateRobotGyroForDegrees(360, "outer");

                leftMotor.setTargetPosition((int) leftMotorTarget);
                rightMotor.setTargetPosition((int) rightMotorTarget);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                //only works if OuterMotorSpeed is 1
                //double reCalcInnerMotorSpeed = leftMotorTarget / rightMotorTarget;

                setMotorPower(leftMotor, 0.20f);
                setMotorPower(rightMotor, 1.00f);

                if (haveBothMotorEncodersReachedTargets(leftMotorTarget, rightMotorTarget)) {
                    setMotorPowers(0.0f, 0.0f);
                    resetMotorEncoders();
                    botstate = State.drop;
                }
                break;

            case turn2:
                singleTurnRobot("Right", 360, State.drop);
                break;

            case drop:
                // Drop object
                //(Close Grippers)
                closeGrippers();
                if(areGrippersClosed()) {
                    openGrippers();
                }
                if (areGrippersOpen()) {
                    botstate = State.done;
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
        update_telemetry();
        update_gamepad_telemetry();
        telemetry.addData("110", "--------------------------------");
        telemetry.addData("111", "State: " + botstate);
        telemetry.addData("120", "--------------------------------");
        telemetry.addData("121", "Left Motor");
        telemetry.addData("122", "      *  Target: " + leftMotorTarget);
        telemetry.addData("123", "      * Current: " + leftMotor.getCurrentPosition());
        telemetry.addData("124", "      *   Power: " + leftMotor.getPower());
        telemetry.addData("125", "      *    Done: " + hasLeftMotorEncoderReachedTarget(leftMotorTarget));
        telemetry.addData("130", "--------------------------------");
        telemetry.addData("131", "Right Motor");
        telemetry.addData("132", "      *  Target: " + rightMotorTarget);
        telemetry.addData("133", "      * Current: " + rightMotor.getCurrentPosition());
        telemetry.addData("134", "      *   Power: " + rightMotor.getPower());
        telemetry.addData("135", "      *    Done: " + hasRightMotorEncoderReachedTarget(rightMotorTarget));
        telemetry.addData("140", "--------------------------------");
        telemetry.addData("141", "Arm Motor");
        telemetry.addData("142", "      *   Power: " + leftArm.getPower());
        telemetry.addData("150", "--------------------------------");
        telemetry.addData("151", "Grippers");
        telemetry.addData("152", "      *  Left Position: " + leftGripper.getPosition());
        telemetry.addData("153", "      * Right Position: " + rightGripper.getPosition());
        telemetry.addData("160", "--------------------------------");
        telemetry.addData("161", "Sweepers");
        telemetry.addData("162", "      *  Left Position: " + leftSweeper.getPosition());
        telemetry.addData("163", "      * Right Position: " + rightSweeper.getPosition());
        telemetry.addData("990", "--------------------------------");
        telemetry.addData("991", "Version: " + Version);

    } // End of start

    // Ending the Autonomous Program
    @Override
    public void stop() {

    } // End of stop

    /**
     * Custom Methods for our Autonomous Program
     */

    void driveForward(double power)
    {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    boolean driveForwardForTime(double power, double targetTime)
    {
        driveForward(power);
        return targetTimeReached(targetTime);
    }

    boolean driveBackwardForTime(double power, double targetTime)
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(-power);
        return targetTimeReached(targetTime);
    }

    boolean turnRightForTime(double power, double targetTime)
    {
        rightMotor.setPower(power);
        leftMotor.setPower(-power);
        return targetTimeReached(targetTime);
    }

    boolean turnLeftForTime(double power, double targetTime)
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(power);
        return targetTimeReached(targetTime);
    }

    boolean liftArmBackwardForTime(double power, double targetTime)
    {
        leftArm.setPower(-power);
        return targetTimeReached(targetTime);
    }

    boolean liftArmForwardForTime(double power, double targetTime)
    {
        leftArm.setPower(power);
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
    void runUsingEncoders ()
    {
        if (leftMotor != null)
        {
            leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        if (rightMotor != null)
        {
            rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    /**
     * Simple Method for resting DcMotor Encoders
     */
    void resetMotorEncoders ()
    {
        if (leftMotor != null)
        {
            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
        if (rightMotor != null)
        {
            rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    };

    /**
     * Simple Method for resting a specific DcMotor Encoders by name of Motor
     */
    void resetMotorEncoder (DcMotor Motor)
    {
        if (Motor != null)
        {
            Motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    };

    /**
     * Checking to see if the Both Motor Encoders Reached Targets
     */
    boolean haveBothMotorEncodersReachedTargets( double leftMotorTarget, double rightMotorTarget)
    {
        boolean BothMotorEncodersReachedTargets = false;
        if (hasLeftMotorEncoderReachedTarget(leftMotorTarget) &&
                hasRightMotorEncoderReachedTarget(rightMotorTarget))
        {
            BothMotorEncodersReachedTargets = true;
        }
        return BothMotorEncodersReachedTargets;
    }

    /**
     * Checking to see if the Left Motor Encoders Reached Target
     */
    boolean hasLeftMotorEncoderReachedTarget (double leftMotorTicks)
    {
        boolean LeftMotorEncoderReachedTarget = false;
        if (leftMotor != null)
        {
            if (Math.abs (leftMotorTicks - leftMotor.getCurrentPosition()) < EncoderTolerance)
            {
                LeftMotorEncoderReachedTarget = true;
            }
        }
        return LeftMotorEncoderReachedTarget;
    }

    /**
     * Checking to see if the Right Motor Encoders Reached Target
     */
    boolean hasRightMotorEncoderReachedTarget (double rightMotorTicks)
    {
        boolean RightMotorEncoderReachedTarget = false;
        if (rightMotor != null)
        {
            if (Math.abs (rightMotorTicks - rightMotor.getCurrentPosition()) < EncoderTolerance)
            {
                RightMotorEncoderReachedTarget = true;
            }
        }
        return RightMotorEncoderReachedTarget;
    }

    /**
     * Have BothMotorEncoders been reset to Zero
     */
    boolean areBothMotorEncodersZero () {
        boolean BothMotorEncodersZero = false;
        if (isLeftMotorEncoderZero() && isRigthMotorEncoderZero()) {
            BothMotorEncodersZero = true;
        }
        return BothMotorEncodersZero;
    }
    // Checking to see if the LeftMotorEncoder is Zero
    boolean isLeftMotorEncoderZero ()
    {
        boolean LeftMotorEncoderZero = false;
        if (leftMotor.getCurrentPosition() == 0)
        {
            LeftMotorEncoderZero = true;
        }
        return LeftMotorEncoderZero;
    }

    // Checking to see if the RightMotorEncoder is Zero
    boolean isRigthMotorEncoderZero ()
    {
        boolean RightMotorEncoderZero = false;
        if (rightMotor.getCurrentPosition () == 0)
        {
            RightMotorEncoderZero = true;
        }
        return RightMotorEncoderZero;
    }

    /**
     * Simple Method for setting the DcMotor power by name of Motor
     */
    void setMotorPowers (double leftPower, double rightPower)

    {
        if (leftMotor != null)
        {
            leftMotor.setPower(leftPower);
        }
        if (rightMotor != null)
        {
            rightMotor.setPower(rightPower);
        }

    }

    /**
     * Simple Method for setting the DcMotor power by name of Motor
     */
    void setMotorPower (DcMotor Motor, double Power)

    {
        if (Motor != null)
        {
            Motor.setPower(Power);
        }
    }

    /**
     * Simple Method to Close Grippers
     */
    void closeGrippers()
    {
        leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);
    }

    boolean areGrippersClosed()
    {
        boolean GrippersClosed = false;
        if((leftGripper.getPosition() == LEFT_GRIP_CLOSED_POSITION)&&
                (rightGripper.getPosition() == RIGHT_GRIP_CLOSED_POSITION))
        {
            GrippersClosed = true;
        }
        return GrippersClosed;
    }

    /**
     * Simple Method to Open Grippers
     */
    void openGrippers()
    {
        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);
    }

    boolean areGrippersOpen()
    {
        boolean GrippersOpen = false;
        if((leftGripper.getPosition() == LEFT_GRIP_OPEN_POSITION)&&
                (rightGripper.getPosition() == RIGHT_GRIP_OPEN_POSITION))
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
    double calculateRobotGyroForDegrees(double Degrees, String WheelLocation)
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
    void RobotMove(int Distance, String Direction, State NextState)
    {
        double leftMotorTicks = driveInchesWithEncoder(Distance);
        double rightMotorTicks = driveInchesWithEncoder(Distance);

        leftMotor.setTargetPosition((int) leftMotorTicks);
        rightMotor.setTargetPosition((int) rightMotorTicks);

        leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(-1.0);
        rightMotor.setPower(-1.0);

        if(hasLeftMotorEncoderReachedTarget(leftMotorTicks))
        {
            setMotorPower(leftMotor,0.0f);
            resetMotorEncoder(leftMotor);
            leftMotorFinished = true;
        }
        if(hasRightMotorEncoderReachedTarget(rightMotorTicks))
        {
            setMotorPower(rightMotor, 0.0f);
            resetMotorEncoder(rightMotor);
            rightMotorFinished = true;
        }
        if((leftMotorFinished)&&(rightMotorFinished))
        {
            leftMotorFinished = false;
            rightMotorFinished = false;
            botstate = NextState;
        }
    }

    void singleTurnRobot(String Direction, int Degrees, State NextState){

        resetMotorEncoders();
        runUsingEncoders();

        double leftMotorTarget = 0;
        float leftMotorPower = 0.0f;

        double rightMotorTarget = 0;
        float rightMotorPower = 0.0f;

        if(Direction.equalsIgnoreCase("left")) {
            leftMotorTarget = calculateRobotGyroForDegrees(Degrees, "inner");
            leftMotorPower = 0.06f;
            rightMotorTarget = calculateRobotGyroForDegrees(Degrees, "outer");
            rightMotorPower = 1.00f;
        }
        if(Direction.equalsIgnoreCase("right")){
            leftMotorTarget = calculateRobotGyroForDegrees(Degrees, "outer");
            leftMotorPower = 1.00f;
            rightMotorTarget = calculateRobotGyroForDegrees(Degrees, "inner");
            rightMotorPower = 0.06f;
        }
        leftMotor.setTargetPosition((int) leftMotorTarget);
        rightMotor.setTargetPosition((int) rightMotorTarget);

        leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        setMotorPower(leftMotor, leftMotorPower);
        setMotorPower(rightMotor, rightMotorPower);

        if (haveBothMotorEncodersReachedTargets(leftMotorTarget, rightMotorTarget)) {
            setMotorPowers(0.0f, 0.0f);
            resetMotorEncoders();
            botstate = NextState
            ;
        }
    }
}
