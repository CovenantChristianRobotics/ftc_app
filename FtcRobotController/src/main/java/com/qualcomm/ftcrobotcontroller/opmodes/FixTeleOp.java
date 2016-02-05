package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 2/4/2016.
 */
public class FixTeleOp extends OpMode {
    Servo rightOmni;
    Servo leftOmni;

    @Override
    public void init(){
        rightOmni = hardwareMap.servo.get("rOmniPinion");
        leftOmni = hardwareMap.servo.get("lOmniPinion");
        rightOmni.setPosition(0.5);
        leftOmni.setPosition(0.5);
    }

    @Override
    public void loop(){
        rightOmni.setPosition((gamepad2.right_stick_y / 2.0) + 0.5);
        leftOmni.setPosition((gamepad2.left_stick_y / 2.0) + 0.5);
    }

    @Override
    public void stop(){
    }
}
