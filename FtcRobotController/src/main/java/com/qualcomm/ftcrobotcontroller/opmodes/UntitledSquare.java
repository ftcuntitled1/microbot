package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public class UntitledSquare extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        for(int count=0; count<4; count++) {
            //go foward for 1 second
            leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);

            sleep(1000);

            //stop motors for .5 seconds
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            sleep(500);

            //tank turn right for 1.5 seconds
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);

            sleep(1500);

            //stop motors for .5 seconds
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            sleep(500);

        }
        //
        leftMotor.setPowerFloat();
        rightMotor.setPowerFloat();

    }
}
