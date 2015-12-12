package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by cchsrobochargers on 12/10/15.
 */
public class CCHS4507TeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    ColorSensor ColorSense;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;
    //switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    boolean nearMountainFlag = false;
    boolean redAllianceFlag = false;



    @Override
    public void init() {
        //Ints hardware
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");
        //servos
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
        ColorSense = hardwareMap.colorSensor.get("color");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        nearMountainFlag = nearMountainSwitch.getState();
        //set up motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }

    @Override
    public void loop() {
        //getting the gamepad values on the y axis
        float right = gamepad1.right_stick_y;
        float left = gamepad1.left_stick_y;
        float lifterMotor = gamepad2.right_stick_y;

        motorRight.setPower(right);
        motorLeft.setPower(left);
        trackLifter.setPower(lifterMotor);

    }
}
