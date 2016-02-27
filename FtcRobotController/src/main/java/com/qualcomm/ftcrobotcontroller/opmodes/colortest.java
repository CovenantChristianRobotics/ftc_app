package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Robotics on 1/23/2016.
 */
public class colortest extends OpMode {

    ColorSensor colorSense;
    ColorSensor otherOne;

    float hsvValues[] = {0F,0F,0F};



    @Override
    public void init() {
        colorSense = hardwareMap.colorSensor.get("mr");
        colorSense.setI2cAddress(0x42);
        colorSense.enableLed(true);
        otherOne = hardwareMap.colorSensor.get("x");
//        otherOne.setI2cAddress(0x40);
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

        Color.RGBToHSV(otherOne.red(), otherOne.green(), otherOne.blue(), hsvValues);

        telemetry.addData("red", otherOne.red());
        telemetry.addData("blue", otherOne.blue());
        telemetry.addData("green", otherOne.green());
        telemetry.addData("alpha", otherOne.alpha());
//        telemetry.addData("?", otherOne.argb());
        telemetry.addData("hue", hsvValues[0]);

        telemetry.addData("redB", colorSense.red());
        telemetry.addData("blueB", colorSense.blue());
        telemetry.addData("HueB", colorSense.argb());
        telemetry.addData("alphaB", colorSense.alpha());
    }

    @Override
    public void stop(){
    }
}

