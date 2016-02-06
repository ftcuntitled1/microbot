package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcguy on 10/18/2015.
 */
public class Everything extends OpMode {
    final double LEFT_GRIP_OPEN_POSITION = 0.0;
    final double LEFT_GRIP_CLOSED_POSITION = 1.0;
    final double RIGHT_GRIP_OPEN_POSITION = 1.0;
    final double RIGHT_GRIP_CLOSED_POSITION = 0.0;

    final double LEFT_SWEEP_OPEN_POSITION = 0.0;
    final double LEFT_SWEEP_CLOSED_POSITION = 1.0;
    final double RIGHT_SWEEP_OPEN_POSITION = 1.0;
    final double RIGHT_SWEEP_CLOSED_POSITION = 0.0;

    DcMotor leftarm;

    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo leftGripper;
    Servo rightGripper;

    Servo leftSweeper;
    Servo rightSweeper;

    @Override
    public void init () {
        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");

        leftSweeper = hardwareMap.servo.get("left_sweeper");
        rightSweeper = hardwareMap.servo.get("right_sweeper");

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftarm = hardwareMap.dcMotor.get("left_arm");

        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);

    }
    @Override
    public void loop () {
        if (gamepad2.x) {
            leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
            rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);
        }
        if (gamepad2.b) {
            leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
            rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);
        }

        if(gamepad1.right_bumper) {
            leftSweeper.setPosition(LEFT_SWEEP_OPEN_POSITION);
        } else {
            leftSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
        }
        if(gamepad1.left_bumper) {
            rightSweeper.setPosition(RIGHT_SWEEP_OPEN_POSITION);
        } else {
            rightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);
        }

        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;

        float leftarmY = -gamepad2.left_stick_y;

        float leftarmYPower = leftarmY/4;

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        leftarm.setPower(leftarmYPower);

    }}

