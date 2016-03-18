package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Robotics on 3/17/2016.
 */
public class demoboard extends OpMode {

    DcMotor ultrasonicControlled;
    DcMotor potControlled;
    Servo touchControlled;
    UltrasonicSensor uCtlr;
    AnalogInput potCtlr;
    TouchSensor servoCtlr;
    ColorSensor color;

    @Override
    public void init() {
        ultrasonicControlled = hardwareMap.dcMotor.get("uMotor");
        potControlled = hardwareMap.dcMotor.get("tMotor");
        touchControlled = hardwareMap.servo.get("tServo");
        uCtlr = hardwareMap.ultrasonicSensor.get("ultraMotor");
        potCtlr = hardwareMap.analogInput.get("potMotor");
        servoCtlr = hardwareMap.touchSensor.get("touchServo");
        color = hardwareMap.colorSensor.get("hT");
    }

    @Override
    public void loop() {
        if(uCtlr.getUltrasonicLevel() < 150 && uCtlr.getUltrasonicLevel() > 10){
            ultrasonicControlled.setPower(uCtlr.getUltrasonicLevel() / 255);
        } else {
            ultrasonicControlled.setPower(0.0);
        }
        potControlled.setPower(potCtlr.getValue() / 255);
        touchControlled.setPosition(servoCtlr.getValue());

        telemetry.addData("Alpha", color.alpha());
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
    }



}
