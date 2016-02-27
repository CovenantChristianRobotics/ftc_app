package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Robotics on 1/23/2016.
 */
public class colortest extends OpMode {

    ColorSensor colorSense;
    ColorSensor otherOne;

    @Override
    public void init() {
        colorSense = hardwareMap.colorSensor.get("mr");
        colorSense.setI2cAddress(0x42);
        colorSense.enableLed(true);
        otherOne = hardwareMap.colorSensor.get("x");
        otherOne.enableLed(false);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            colorSense.enableLed(false);
        } else if (gamepad1.b) {
            colorSense.enableLed(true);
        }

        if (gamepad2.a) {
            otherOne.enableLed(false);
        } else if (gamepad2.b) {
            otherOne.enableLed(true);
        }

        telemetry.addData("red", colorSense.red());
        telemetry.addData("blue", colorSense.blue());
        telemetry.addData("green", colorSense.green());
        telemetry.addData("alpha", colorSense.alpha());
        telemetry.addData("?", colorSense.argb());
    }

    @Override
    public void stop(){
    }
}

