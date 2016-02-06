package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ftcguy on 11/8/2015.
 */
public class UntitledAutoPush extends OpMode{

    DcMotor rightMotor;
    DcMotor leftMotor;
    ElapsedTime time;

    static final double forwardTime = 1.0;
    static final double turnTime = 1.0;
    int count = 0;

    enum State {drivingStraight, turning, done};
    State state;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        time = new ElapsedTime();
        state = State.drivingStraight;
    }

    @Override
    public void loop () {
        double currentTime = time.time();
        switch (state) {
            case drivingStraight:
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
                if (currentTime > forwardTime) {

                }
        }
    }

}
