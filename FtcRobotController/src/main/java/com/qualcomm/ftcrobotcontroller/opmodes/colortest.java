package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Robotics on 1/23/2016.
 */
public class colortest extends OpMode {

    ColorSensor colorSense;

    @Override
    public void init() {
        colorSense = hardwareMap.colorSensor.get("mr");
        colorSense.enableLed(true);
    }

    @Override
    public void loop() {
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

