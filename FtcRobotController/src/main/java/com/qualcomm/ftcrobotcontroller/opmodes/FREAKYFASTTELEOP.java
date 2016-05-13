package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by CCHSRobotics on 5/6/2016.
 */
public class FREAKYFASTTELEOP extends OpMode {
    DcMotor motorLeft;
    DcMotor motorRight;

    @Override
    public void init() {
        //Ints hardware
        motorRight = hardwareMap.dcMotor.get("r");
        motorLeft = hardwareMap.dcMotor.get("l");
    }

    @Override
    public void loop() {
        double rightDrive;
        double leftDrive;

        rightDrive = -gamepad1.right_stick_y;
        leftDrive = -gamepad1.left_stick_y;
    }
}
