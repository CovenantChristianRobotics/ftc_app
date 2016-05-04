package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

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
    Integer badDist;

    @Override
    public void init() {
        ultrasonicControlled = hardwareMap.dcMotor.get("uMotor");
        potControlled = hardwareMap.dcMotor.get("tMotor");
        touchControlled = hardwareMap.servo.get("tServo");
        uCtlr = hardwareMap.ultrasonicSensor.get("ultraMotor");
        potCtlr = hardwareMap.analogInput.get("potMotor");
        servoCtlr = hardwareMap.touchSensor.get("touchServo");
        color = hardwareMap.colorSensor.get("hT");
        badDist = 0;
    }

    @Override
    public void loop() {
        if(uCtlr.getUltrasonicLevel() < 150.0 && uCtlr.getUltrasonicLevel() > 10.0){
            ultrasonicControlled.setPower(uCtlr.getUltrasonicLevel() / 255.0);
            badDist = 0;
        } else {
            if (badDist < 3) {
                badDist++;
            } else {
                ultrasonicControlled.setPower(0.0);
            }
        }
        potControlled.setPower(Range.clip((potCtlr.getValue() - 512) / 512.0, -1.0, 1.0));
        touchControlled.setPosition(servoCtlr.getValue());

        telemetry.addData("Alpha", color.alpha());
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
    }



}
