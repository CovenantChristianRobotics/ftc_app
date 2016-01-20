package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 1/19/2016.
 */
public class CCHS5256Autonomous extends OpMode {

    enum MoveState {
        STARTMOVE, MOVING, DELAYSETTINGS, DELAY
    }

    // DC Motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor chinUp;
    DcMotor endGameLights;
    // Servos
    Servo beaconPinion;
    Servo beaconPusher;
    Servo climberDumper;
    Servo ultraSenseServo;
    Servo leftOmniPinion;
    Servo rightOmniPinion;
    // Sensors
    GyroSensor gyroSense;

    @Override
    public void init() {
        // DC Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        chinUp = hardwareMap.dcMotor.get("chinUp");
        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        // DC Motor Settings
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        // Servos
        beaconPinion = hardwareMap.servo.get("beaconPinion");
        beaconPusher = hardwareMap.servo.get("beaconPusher");
        climberDumper = hardwareMap.servo.get("climber_dumper");
        ultraSenseServo = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        // Servo Settings
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        // Sensors
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        while (gyroSense.isCalibrating()) {
        }


    }

    @Override
    public void loop() {

    }

    @Override
    public void stop () {
    }


}
