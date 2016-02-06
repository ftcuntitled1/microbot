

package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

public class UntitledAuto2 extends OpMode{

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

    // The distance between the left and right tires
    double AxleWidth = 15.5;

    // Add a buffer to the inside turn radius to prevent freezing
    double AxleWidthBuffer = 1;

    // Encoder Tolerance +/- based on ticks not inches
    // Also helps with the prevention of freezing when comparing TargetEncoderTicks
    double EncoderTolerance = 4.0;

    // Test Variables to see if Motors have reached the TargetEncoderTicks
    boolean leftMotorDone = false;
    boolean rightMotorDone = false;

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
        startup, turn1, foward1, turn2, lift, back2, drop, flippers, done
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

        // Resetting the Motor Encoders to make sure we always start at 0 ticks.
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

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

                //reset_drive_encoders ();
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                botstate = State.turn1;

                break;

            case turn1:

                // turn left 45 degrees
                //final static double DISTANCEturn1 = 12.964;

                double leftMotorTicksTurnLeft = turnForDegrees(45, "Inner");
                double rightMotorTicksTurnLeft = turnForDegrees(45, "Outer");

                leftMotor.setTargetPosition((int) leftMotorTicksTurnLeft );
                rightMotor.setTargetPosition((int) rightMotorTicksTurnLeft);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(0.2);
                rightMotor.setPower(1.0);

                if (Math.abs (leftMotorTicksTurnLeft - leftMotor.getCurrentPosition()) < EncoderTolerance)
                {
                    leftMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    leftMotorDone = true;
                }
                if (Math.abs (rightMotorTicksTurnLeft - rightMotor.getCurrentPosition()) < EncoderTolerance)
                {
                    rightMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotorDone = true;
                }
                if ((leftMotorDone) && (rightMotorDone)){
                    leftMotorDone = false;
                    rightMotorDone = false;
                    botstate = State.foward1;
                }

                break;

            case foward1:

                // Go forward 85.00 inches
                //final static double DISTANCEfoward1 = 85.00;

                double leftMotorTicksForward = driveInchesWithEncoder(85.0);
                double rightMotorTicksForward = driveInchesWithEncoder(85.0);

                leftMotor.setTargetPosition((int) leftMotorTicksForward);
                rightMotor.setTargetPosition((int) rightMotorTicksForward);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(1.0);
                rightMotor.setPower(1.0);

                if (Math.abs (leftMotorTicksForward - leftMotor.getCurrentPosition ()) < EncoderTolerance)
                {
                    leftMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    leftMotorDone = true;
                }
                if (Math.abs (rightMotorTicksForward - rightMotor.getCurrentPosition ()) < EncoderTolerance)
                {
                    rightMotor.setPower(0.0);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotorDone = true;
                }
                if ((leftMotorDone) && (rightMotorDone)){
                    leftMotorDone = false;
                    rightMotorDone = false;
                    botstate = State.turn2;
                }

                break;

            case turn2:

                // Turn left 135 degrees
                //final static double DISTANCEturn2 = 38.893; //135 degrees

                double leftMotorTicks_turn2 = turnForDegrees(135, "Inner");
                double rightMotorTicks_turn2 = turnForDegrees(135, "Outer");

                leftMotor.setTargetPosition((int) leftMotorTicks_turn2 );
                rightMotor.setTargetPosition((int) rightMotorTicks_turn2);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(-1.0);
                rightMotor.setPower(-0.2);

                if (Math.abs (leftMotor.getCurrentPosition ()) < leftMotorTicks_turn2)
                {
                    leftMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    leftMotorDone = true;
                }
                if (Math.abs (rightMotor.getCurrentPosition ()) < rightMotorTicks_turn2)
                {
                    rightMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotorDone = true;
                }
                if ((leftMotorDone) && (rightMotorDone)){
                    leftMotorDone = false;
                    rightMotorDone = false;
                    botstate = State.back2;
                }

                break;

            case back2:

                // Go backwards 23 inches
                //final static double DISTANCEback1 = 23.00;

                double leftMotorTicksBackToWall = driveInchesWithEncoder(85.0);
                double rightMotorTicksBackToWall = driveInchesWithEncoder(85.0);

                leftMotor.setTargetPosition((int) leftMotorTicksBackToWall);
                rightMotor.setTargetPosition((int) rightMotorTicksBackToWall);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(-1.0);
                rightMotor.setPower(-1.0);

                if (Math.abs (leftMotorTicksBackToWall - leftMotor.getCurrentPosition ()) < EncoderTolerance)
                {
                    leftMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    leftMotorDone = true;
                }
                if (Math.abs (rightMotorTicksBackToWall - rightMotor.getCurrentPosition ()) < EncoderTolerance)
                {
                    rightMotor.setPower(0.0);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotorDone = true;
                }
                if ((leftMotorDone) && (rightMotorDone)){
                    leftMotorDone = false;
                    rightMotorDone = false;
                    botstate = State.lift;
                }

                break;

            case lift:

                // Move arm motor up unknown ticks

                if ( liftArmForwardForTime(0.1, 10)) {
                    leftArm.setPower(0.0);
                    targetTimeReached(1);
                    botstate = State.drop;
                }

                break;

            case drop:

                // Drop object
                //(Open Grippers)

                double currentTimedrop = time.time();

                leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
                rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);

                if (currentTimedrop < dropTime) {

                    //do nothing but wait or sleep
                    return;

                } else {

                    leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
                    rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);

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

        telemetry.addData("19", "State: " + botstate);

        telemetry.addData("20", "Right Motor");
        telemetry.addData("21", "      * Target: " + rightMotor.getTargetPosition());
        telemetry.addData("22", "      * Position: " + rightMotor.getCurrentPosition());
        telemetry.addData("23", "      * Power: " + rightMotor.getPower());

        telemetry.addData("24", "Left Motor");
        telemetry.addData("25", "      * Target: " + leftMotor.getTargetPosition());
        telemetry.addData("26", "      * Position: " + leftMotor.getCurrentPosition());
        telemetry.addData("27", "      * Power: " + leftMotor.getPower());

        telemetry.addData("28", "Arm Motor");
        telemetry.addData("29", "      * Power: " + leftArm.getPower());

        telemetry.addData("30", "Grippers");
        telemetry.addData("31", "      * Left Position: " + leftGripper.getPosition());
        telemetry.addData("32", "      * Right Position: " + rightGripper.getPosition());

        telemetry.addData("33", "Sweepers");
        telemetry.addData("34", "      * Left Position: " + leftSweeper.getPosition());
        telemetry.addData("35", "      * Right Position: " + rightSweeper.getPosition());




    } // Enf of loop

    // Ending the Autonomous Program
    @Override
    public void stop() {

    }

    /**
     * Custom Methods for our Autonomous Program
     */

    public void driveForward(double power)
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(-power);
    }

    public boolean driveForwardForTime(double power, double targetTime)
    {
        driveForward(power);
        return targetTimeReached(targetTime);
    }

    public boolean driveBackwardForTime(double power, double targetTime)
    {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
        return targetTimeReached(targetTime);
    }

    public boolean turnRightForTime(double power, double targetTime)
    {
        rightMotor.setPower(power);
        leftMotor.setPower(-power);
        return targetTimeReached(targetTime);
    }

    public boolean turnLeftForTime(double power, double targetTime)
    {
        rightMotor.setPower(-power);
        leftMotor.setPower(power);
        return targetTimeReached(targetTime);
    }

    public boolean liftArmBackwardForTime(double power, double targetTime)
    {
        leftArm.setPower(-power);
        return targetTimeReached(targetTime);
    }

    public boolean liftArmForwardForTime(double power, double targetTime)
    {
        leftArm.setPower(power);
        return targetTimeReached(targetTime);
    }

    public boolean targetTimeReached( double targetTime)
    {

        if (!runTimerStarted) {
            runTimerStarted = true;
            startTime = getRuntime();
            return false;
        } else {
            boolean result = (getRuntime() - startTime) >= targetTime;
            if (result) {
                runTimerStarted = false;
                return result;
            }
            return true;
        }
    }

    /**
     * This Method will calculate the number of MotorTicksToTurn for the motor to turn
     * based on the given parameter of distanceInches when called
     * param distanceInches
     * return
     */
    public double driveInchesWithEncoder (double distanceInches)
    {
        double WheelRotations = distanceInches / WheelCircumference;
        return EncoderTicksPerRotation * WheelRotations * GearRatio;
    }

    /**
     * This Method will return the value for the MotorTicksToTurnOuter or MotorTicksToTurnInner
     * for the motor to turn based on the Gyro Degrees when called adn dynamically change the
     * Inner and Outer based on the WheelLocation
     * param Degrees
     * param WheelLocation
     * return
     */
    public double turnForDegrees (double Degrees, String WheelLocation)
    {


        if (WheelLocation.equalsIgnoreCase("outer"))
        {
            // Set to the AxleWidth plus the AxleWidthBoffer, if is the outside wheel turning
            double RobotTurnDiameterOuter = AxleWidth + AxleWidthBuffer * 2;
            // Total Circumference in Inches for the Robot Turn Radius of 360
            double RobotTurnCircumferenceOuter = RobotTurnDiameterOuter * Math.PI;
            // The number of WheelRotations to drive the full Robot Turn Radius of 360
            double WheelRotationsOuter = RobotTurnCircumferenceOuter / WheelCircumference;
            // The number of EncoderTicks for the Motor to turn for the full Robot Turn Radius of 360
            double TicksToTurnOuter = WheelRotationsOuter * EncoderTicksPerRotation * GearRatio;
            // Degrees for how far we want to turn out of the 360, so a straight ratio here
            double TurnRatioOuter = Degrees / 360;
            // Now wrapping it up with the Ration * to TicksToTurnOuter to get our partial turn
            // return these results back to the part of the program that called this method
            return TurnRatioOuter * TicksToTurnOuter;
        }
        if (WheelLocation.equalsIgnoreCase("inner")) {
            // Set to the AxleWidthBuffer only, if is the inner wheel turning
            double RobotTurnDiameterInner = AxleWidthBuffer * 2;
            // Total Circumference in Inches for the Robot Turn Radius of 360
            double RobotTurnCircumferenceInner = RobotTurnDiameterInner * Math.PI;
            // The number of WheelRotations to drive the full Robot Turn Radius of 360
            double WheelRotationsInner = RobotTurnCircumferenceInner / WheelCircumference;
            // The number of EncoderTicks for the Motor to turn for the full Robot Turn Radius of 360
            double TicksToTurnInner = WheelRotationsInner * EncoderTicksPerRotation * GearRatio;
            // Degrees for how far we want to turn out of the 360, so a straight ratio here
            double TurnRatioInner = Degrees / 360;
            // Now wrapping it up with the Ratio * TicksToTurnInner to get our partial turn
            // return these results back to the part of the program that called this method
            return TurnRatioInner * TicksToTurnInner;
        }
        else
        {
            //Do Nothing set the
            return 0;
        }
    }


}  // PushBotAuto
