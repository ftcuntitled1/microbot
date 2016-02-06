package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen on 1/12/2016.
 */
public class Untitled1Resources {

    public static void CoreComponents()
    {

        //setting the version for Telemetry Data
        String Version = "1.95";

        // Enter the number of Ticks per single Rotation of Motor (ours is 1440)
        final int EncoderTicksPerRotation = 1440;

        // Calculated gear ratio from motor to wheel
        // Number of Teeth on the Tire Axle Gear divided by the Number of Teeth on the Motor Gear
        final int GearRatio = 2;

        // The Full Diameter of the Wheel Tread in Inches
        // Normally marked on the side of the Tire Tread, but might not be in inches.
        final int WheelDiameter = 4;

        // Calculating the Circumference of the Tire based on the above Variables Provided
        final double WheelCircumference = Math.PI * WheelDiameter;

        // The distance between the left and right tires
        final double AxleWidth = 14.5;

        // Add a buffer to the inside turn radius to prevent freezing
        final double AxleWidthBuffer = 1.0;

        // Encoder Tolerance +/- based on ticks not inches
        // Also helps with the prevention of freezing when comparing TargetEncoderTicks
        final int EncoderTolerance = 4;

        final double startTime;
        final boolean runTimerStarted;

        // Defining some static positions for the Servo Motors
        final double LEFT_GRIP_OPEN_POSITION = 0.0;
        final double LEFT_GRIP_CLOSED_POSITION = 1.0;

        final double RIGHT_GRIP_OPEN_POSITION = 1.0;
        final double RIGHT_GRIP_CLOSED_POSITION = 0.0;

        //final double LEFT_SWEEP_OPEN_POSITION = 0.0;
        final double LEFT_SWEEP_CLOSED_POSITION = 1.0;

        //final double RIGHT_SWEEP_OPEN_POSITION = 1.0;
        final double RIGHT_SWEEP_CLOSED_POSITION = 0.0;

        // Setting the friendly names for our Motors and Servos
        DcMotor leftArm;

        DcMotor leftMotor;
        DcMotor rightMotor;

        Servo leftGripper;
        Servo rightGripper;

        Servo leftSweeper;
        Servo rightSweeper;

        ElapsedTime time;

        final double leftMotorTarget;
        final double rightMotorTarget;

        final double leftMotorPower;
        final double rightMotorPower;
    }

}
