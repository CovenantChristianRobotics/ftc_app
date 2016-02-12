package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
//import java.util.Date;

/**
 * Created by cchsrobochargers on 12/17/15.
 */
public class CCHS4507Test extends OpMode {
    double xHeading;
    double yHeading;
    long gyroReadLast;
    GyroSensor gyroSense;
    int gyroError;
    long now;

    public CCHS4507Test() {
    }

    @Override
    public void init() {
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        xHeading = 0;
        yHeading = 0;
        gyroReadLast = System.currentTimeMillis();
        now = System.currentTimeMillis();

        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
    }

    @Override
    public void loop() {

        if (gyroSense.isCalibrating()) {
            return;
        }
        now = System.currentTimeMillis();
        xHeading = xHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawX();
        yHeading = yHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawY();
        gyroReadLast = now;

        telemetry.addData("gyro", Integer.toString(gyroSense.getHeading()));
        telemetry.addData("Xheading", xHeading);
        telemetry.addData("yHeading", yHeading);
        telemetry.addData("rawX", gyroSense.rawX());
        telemetry.addData("rawY", gyroSense.rawY());

//        Log.i("Current Move", currentMove.toString());
//        Log.i("desiredHeading", Integer.toString(desiredHeading));
//        Log.i("gyro", Integer.toString(gyroSense.getHeading()));
//        Log.i("colorRed", Integer.toString(colorSense.red()));
//        Log.i("colorBlue", Integer.toString(colorSense.blue()));
//        Log.i("colorGreen", Integer.toString(colorSense.green()));
//        Log.i("colorAlpha", Integer.toString(colorSense.alpha()));
//        Log.i("ambientRed", Integer.toString(ambientRed));
//        Log.i("ambientBlue", Integer.toString(ambientBlue));
    }

    @Override
    public void stop() {
    }
}