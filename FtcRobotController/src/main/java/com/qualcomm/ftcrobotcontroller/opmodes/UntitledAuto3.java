

package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Extends the PushBotTelemetry and PushBotHardware classes to provide a basic
 * autonomous operational mode for the Push Bot.
 *
 * @author Tyler Bechard
 * @version 2015-10-31
 */

public class UntitledAuto3 extends PushBotTelemetry {

    //Enter the version of the program
    String botversion = "1.0.0";
    // Enter the number of Ticks per single Rotation of Motor (ours is 1440)
    int EncoderTicksPerRotation = 1440;

    // Calculated gear ratio from motor to wheel
    // Number of Teeth on the Tire Axle Gear divided by the Number of Teeth on the Motor Gear
    double GearRatio = 2;

    // The Full Diameter of the Wheel Tread in Inches
    // Normally marked on the side of the Tire Tread, but might not be in inches.
    int WheelDiameter = 4;

    // Calculating the Circumference of the Tire based on the above Variables Provided
    double WheelCircumference = Math.PI * WheelDiameter;

    // The distance between the left and right tires //Original width 15.5
    double AxleWidth = 14.5;

    // Add a buffer to the inside turn radius to prevent freezing
    double AxleWidthBuffer = 1;

    // Encoder Tolerance +/- based on ticks not inches
    // Also helps with the prevention of freezing when comparing TargetEncoderTicks
    double EncoderTolerance = 4.0;

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

    // Timers to keep track of time for the robot programming run time
    double sweeperTime = 1.0;
    double dropTime = 1.0;
    double liftTime = 1.0;

    // Enumerating the potential list of options or cases for our Switch block.
    enum State
    {
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
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setting and Moving the Servos to a specific start position at startup
        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);

        leftSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
        rightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);

    }

    /**
     * Send preliminary data back to the driver station on the current state and driving motors
     */
    @Override
    public void init_loop()
    {
        resetMotorEncoders();

        telemetry.addData("99", "--------------------------------");
        telemetry.addData("100", "State: " + botstate);
        telemetry.addData("101", "Version: " + botversion);
        telemetry.addData("102", "--------------------------------");
        telemetry.addData("103", "Right Motor");
        telemetry.addData("105", "      * Current: " + rightMotor.getCurrentPosition());
        telemetry.addData("107", "--------------------------------");
        telemetry.addData("108", "Left Motor");
        telemetry.addData("110", "      * Current: " + leftMotor.getCurrentPosition());
        telemetry.addData("112", "--------------------------------");
    }

    /**
     * This is our main loop ( or program the is run when you hit the play button
     * on the drivers station phone and will continue looping until you hit stop
     */
    public void loop() {

        switch(botstate) {

            case startup:
                resetMotorEncoders();
                botstate = State.turn1;
                break;

            case turn1:
                // turn left 45 degrees
                //final static double DISTANCEturn1 = 12.964;
                runUsingEncoders();
                setMotorPowers(1.0f, 0.20f);
                if(have_drive_encoders_reached
                        (
                                turnRobotForDegrees(45,"inner"),
                                turnRobotForDegrees(45,"outer")
                        )
                        )
                {
                    setMotorPowers(0.0f, 0.0f);
                    resetMotorEncoders();
                    botstate = State.turn1_wait;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case turn1_wait:
                if (areBothMotorEncodersZero())
                {
                    botstate = State.done;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case forward1:
                // Go forward 85.00 inches
                //final static double DISTANCEfoward1 = 85.00;
                runUsingEncoders();
                setMotorPowers(1.0f, 1.0f);
                if(have_drive_encoders_reached
                        (
                                driveInchesWithEncoder(85.0),
                                driveInchesWithEncoder(85.0)
                        )
                        )
                {
                    setMotorPowers(0.0f, 0.0f);
                    resetMotorEncoders();
                    botstate = State.forward1_wait;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case forward1_wait:
                if (areBothMotorEncodersZero())
                {
                    botstate = State.turn2;
                }
                else
                {
                   DisplayMotorTelemetry();
                }
                break;

            case turn2:
                // Turn left 135 degrees
                //final static double DISTANCEturn2 = 38.893; //135 degrees
                runUsingEncoders();
                setMotorPowers(1.0f, 1.0f);
                if(have_drive_encoders_reached
                        (
                                turnRobotForDegrees(135,"inner"),
                                turnRobotForDegrees(135,"outer")
                        )
                        )
                {
                    setMotorPowers(0.0f, 0.0f);
                    resetMotorEncoders();
                    botstate = State.turn2_wait;
                }
                else
                {
                   DisplayMotorTelemetry();
                }
                break;

            case turn2_wait:
                if (areBothMotorEncodersZero()) {
                    botstate = State.back2;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case back2:
                // Go backwards 23 inches
                //final static double DISTANCEback1 = 23.00;
                runUsingEncoders();
                setMotorPowers(-1.0f, -1.0f);
                if(have_drive_encoders_reached
                        (
                                driveInchesWithEncoder(23.0),
                                driveInchesWithEncoder(23.0)
                        )
                        )
                {
                    setMotorPowers(0.0f, 0.0f);
                    resetMotorEncoders();
                    botstate = State.back2_wait;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case back2_wait:
                if (areBothMotorEncodersZero()) {
                    botstate = State.lift;
                }
                else
                {
                    DisplayMotorTelemetry();
                }
                break;

            case lift:
                // Move arm motor up unknown ticks
                liftArmForwardForTime(0.1, 10);
                botstate = State.lift_wait;
                break;

            case lift_wait:
                if (leftArm.getPower() == 0) {
                    botstate = State.drop;
                }
                break;

            case drop:
                // Drop object
                //(Close Grippers)
                closeGrippers();
                botstate = State.done;
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


      /*  // Send telemetry data to the driver station.
        update_telemetry ();
        update_gamepad_telemetry();
        telemetry.addData("110", "--------------------------------");
        telemetry.addData("110", "State: " + botstate);
        telemetry.addData("100", "--------------------------------");
        telemetry.addData("120", "Right Motor");
        telemetry.addData("121", "      *  Target: " + rightMotor.getTargetPosition());
        telemetry.addData("122", "      * Current: " + rightMotor.getCurrentPosition());
        telemetry.addData("123", "      *   Power: " + rightMotor.getPower());
        telemetry.addData("100", "--------------------------------");
        telemetry.addData("130", "Left Motor");
        telemetry.addData("131", "      *  Target: " + leftMotor.getTargetPosition());
        telemetry.addData("132", "      * Current: " + leftMotor.getCurrentPosition());
        telemetry.addData("133", "      *   Power: " + leftMotor.getPower());
        telemetry.addData("100", "--------------------------------");
        telemetry.addData("140", "Arm Motor");
        telemetry.addData("141", "      *   Power: " + leftArm.getPower());
        telemetry.addData("100", "--------------------------------");
        telemetry.addData("150", "Grippers");
        telemetry.addData("151", "      *  Left Position: " + leftGripper.getPosition());
        telemetry.addData("152", "      * Right Position: " + rightGripper.getPosition());
        telemetry.addData("100", "--------------------------------");
        telemetry.addData("160", "Sweepers");
        telemetry.addData("161", "      *  Left Position: " + leftSweeper.getPosition());
        telemetry.addData("162", "      * Right Position: " + rightSweeper.getPosition());
        telemetry.addData("100", "--------------------------------");
*/
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
    boolean haveBothMotorEncodersReachedTargets
            ( double leftTargetCount
                    , double rightTargetCount
            )
    {
        boolean BothMotorEncodersReachedTargets = false;
        if (hasLeftMotorEncodersReachedTarget(leftTargetCount) &&
                hasRightMotorEncodersReachedTarget(rightTargetCount))
        {
            BothMotorEncodersReachedTargets = true;
        }
        return BothMotorEncodersReachedTargets;
    }

    /**
     * Checking to see if the Left Motor Encoders Reached Target
     */
    boolean hasLeftMotorEncodersReachedTarget (double MotorTicks)
    {
        boolean LeftMotorEncodersReachedTarget = false;
        if (leftMotor != null)
        {
            if (Math.abs (leftMotor.getCurrentPosition ()) > MotorTicks)
            {
                LeftMotorEncodersReachedTarget = true;
            }
        }
        return LeftMotorEncodersReachedTarget;
    }

    /**
     * Checking to see if the Right Motor Encoders Reached Target
     */
    boolean hasRightMotorEncodersReachedTarget (double MotorTicks)
    {
        boolean RightMotorEncodersReachedTarge = false;
        if (rightMotor != null)
        {
            if (Math.abs (rightMotor.getCurrentPosition ()) > MotorTicks)
            {
                RightMotorEncodersReachedTarge = true;
            }
        }
        return RightMotorEncodersReachedTarge;
    }

    /**
     * Have BothMotorEncoders been reset to Zero
     */
    boolean areBothMotorEncodersZero () {
        boolean l_return = false;
        if (isLeftMotorEncoderZero() && isRigthMotorEncoderZero()) {
            l_return = true;
        }
        return l_return;
    }
    // Checking to see if the LeftMotorEncoder is Zero
    boolean isLeftMotorEncoderZero ()
    {
        boolean LeftMotorEncoderZero = false;
        if (a_left_encoder_count() == 0)
        {
            LeftMotorEncoderZero = true;
        }
        return LeftMotorEncoderZero;
    }

    // Checking to see if the RightMotorEncoder is Zero
    boolean isRigthMotorEncoderZero ()
    {
        boolean RightMotorEncoderZero = false;
        if (a_right_encoder_count() == 0)
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
            leftMotor.setPower (leftPower);
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
            Motor.setPower (Power);
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

    /**
     * Simple Method to Open Grippers
     */
    void openGrippers()
    {
        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);
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
        return EncoderTicksPerRotation * WheelRotations * GearRatio;
    }

    /**
     * This Method will return the value for the MotorTicksToTurnOuter or MotorTicksToTurnInner
     * for the motor to turn based on the Gyro Degrees when called adn dynamically change the
     * Inner and Outer based on the WheelLocation
     * param Degrees (0 - 360)
     * param WheelLocation (inner or outer)
     * return - the results in MotorTicks
     */
    double turnRobotForDegrees(double Degrees, String WheelLocation)
    {

        double RobotTurnDiameter = 0.0;
        if (WheelLocation.equalsIgnoreCase("outer"))
        {
            // Set to the AxleWidth plus the AxleWidthBoffer, if is the outside wheel turning
            RobotTurnDiameter = AxleWidth + AxleWidthBuffer * 2;
        }
        if (WheelLocation.equalsIgnoreCase("inner"))
        {
            // Set to the AxleWidthBuffer only, if is the inner wheel turning
            RobotTurnDiameter = AxleWidthBuffer * 2;
        }
        // Total Circumference in Inches for the Robot Turn Radius of 360
        double RobotTurnCircumference = RobotTurnDiameter * Math.PI;
        // The number of WheelRotations to drive the full Robot Turn Radius of 360
        double WheelRotations = RobotTurnCircumference / WheelCircumference;
        // The number of EncoderTicks for the Motor to turn for the full Robot Turn Radius of 360
        double TurnCircumferenceMotorTicks = WheelRotations * EncoderTicksPerRotation * GearRatio;
        // Degrees for how far we want to turn out of the 360, so a straight ratio here
        double TurnRatio = Degrees / 360;
        // Now wrapping it up with the Ration * to TurnCircumferenceTicks to get our partial turn
        // return these results back to the part of the program that called this method
        return TurnRatio * TurnCircumferenceMotorTicks;
        }

    /**
     * Lets make this robot move based on
     * param Distance - (0 - 999 Inches)
     * param Direction - (Forward, Backwards, RTurn, LTurn)
     */
    void RobotMove(int Distance, String Direction, State NextState) {
        double leftMotorTicks = driveInchesWithEncoder(Distance);
        double rightMotorTicks = driveInchesWithEncoder(Distance);

        leftMotor.setTargetPosition((int) leftMotorTicks);
        rightMotor.setTargetPosition((int) rightMotorTicks);

        //leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(-1.0);
        rightMotor.setPower(-1.0);

        if (hasLeftMotorEncodersReachedTarget(leftMotorTicks)) {
            setMotorPower(leftMotor, 0.0f);
            resetMotorEncoder(leftMotor);
            leftMotorFinished = true;
        }
        if (hasRightMotorEncodersReachedTarget(rightMotorTicks)) {
            setMotorPower(rightMotor, 0.0f);
            resetMotorEncoder(rightMotor);
            rightMotorFinished = true;
        }
        if ((leftMotorFinished) && (rightMotorFinished)) {
            leftMotorFinished = false;
            rightMotorFinished = false;
            botstate = NextState;
        }
    }


        //displays drive motor telemetry

    public void DisplayMotorTelemetry() {
        telemetry.addData("102", "--------------------------------");
        telemetry.addData("103", "Right Motor");
        telemetry.addData("104", "      *  Target: " + rightMotor.getTargetPosition());
        telemetry.addData("105", "      * Current: " + rightMotor.getCurrentPosition());
        telemetry.addData("106", "      *   Power: " + rightMotor.getPower());
        telemetry.addData("107", "--------------------------------");
        telemetry.addData("108", "Left Motor");
        telemetry.addData("109", "      *  Target: " + leftMotor.getTargetPosition());
        telemetry.addData("110", "      * Current: " + leftMotor.getCurrentPosition());
        telemetry.addData("111", "      *   Power: " + leftMotor.getPower());
        telemetry.addData("112", "--------------------------------");
    }

}  // PushBotAuto
