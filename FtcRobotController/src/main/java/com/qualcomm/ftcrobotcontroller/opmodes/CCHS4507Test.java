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
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor motor5;
    DcMotor motor6;
    Servo trapDoorRight;
    Servo trapDoorBro;


    public CCHS4507Test() {

    }

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        motor4 = hardwareMap.dcMotor.get("motor_4");
        motor5 = hardwareMap.dcMotor.get("motor_5");
        motor6 = hardwareMap.dcMotor.get("motor_6");
        trapDoorRight = hardwareMap.servo.get("servo_1");
        trapDoorBro = hardwareMap.servo.get("servo_2");
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motor2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

    @Override
    public void loop() {
        double rightDrive;
        double leftDrive;
        double moveArm;
        double moveArmUp;
        double alsoMoveArmUp;
        leftDrive = gamepad1.right_stick_y;
        rightDrive = -gamepad1.left_stick_y;
        moveArm = -gamepad2.left_stick_x;
        moveArmUp = -gamepad2.left_stick_y;
        alsoMoveArmUp = -gamepad2.left_stick_y;
        motor1.setPower(rightDrive);
        motor2.setPower(leftDrive);
        motor3.setPower(moveArm/3);
        motor4.setPower(moveArmUp/3);
        motor6.setPower(alsoMoveArmUp/3);
        if (gamepad2.a){
            trapDoorRight.setPosition(0.0);
            trapDoorBro.setPosition(1.0);
        }
        if (gamepad2.y){
            trapDoorRight.setPosition(1.0);
            trapDoorBro.setPosition(0.0);
        }
        if (gamepad2.right_bumper) {
            motor5.setPower(0.8);
        }
        if (gamepad2.x) {
            motor5.setPower(0.0);
        }
        if (gamepad2.left_bumper) {
            motor5.setPower(-0.8);
        }




       // if (gyroSense.isCalibrating()) {
        //    return;
       // }
       // now = System.currentTimeMillis();
      //  xHeading = xHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawX();
      //  yHeading = yHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawY();
       // gyroReadLast = now;

      //  telemetry.addData("gyro", Integer.toString(gyroSense.getHeading()));
       // telemetry.addData("Xheading", xHeading);
       // telemetry.addData("yHeading", yHeading);
       // telemetry.addData("rawX", gyroSense.rawX());
       // telemetry.addData("rawY", gyroSense.rawY());

//        Log.i("Current Move", currentMove.toString());
//        Log.i("desiredHeading", Integer.toString(desiredHeading));
//        Log.i("gyro", Integer.toString(gyroSense.getHeading()));
//        Log.i("colorRed", Inte;ger.toString(colorSense.red()));
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