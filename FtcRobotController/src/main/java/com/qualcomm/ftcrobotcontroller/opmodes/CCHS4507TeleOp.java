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

/**
 * Created by cchsrobochargers on 12/10/15.
 */
public class CCHS4507TeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    DcMotor armExtend;

    //servos
    //Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    Servo climberTriggerLeft;
    Servo climberTriggerRight;
//    Servo climberDoor;
    Servo cowCatcher;
    Servo armLock;
    Servo trackLock;
    //Servo zipTieSweeper;

    //Sensors
    ColorSensor ColorSense;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;
    TouchSensor liftCheck;

    //switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    boolean nearMountainFlag = false;
    boolean redAllianceFlag = false;
    boolean cowCatcherUp = false;
    int trackLifterUp = 0;
    int trackLifterDown = 1170 * 2;


    @Override
    public void init() {
        //Ints hardware
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");
        armExtend = hardwareMap.dcMotor.get("armExtend");

        //servos
        //servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
//        climberDoor = hardwareMap.servo.get("climberDoor");
        servoDist = hardwareMap.servo.get("servoDist");
        climberTriggerLeft = hardwareMap.servo.get("trigLeft");
        climberTriggerRight = hardwareMap.servo.get("trigRight");
        cowCatcher = hardwareMap.servo.get("cowCatcher");
        armLock = hardwareMap.servo.get("armLock");
        trackLock = hardwareMap.servo.get("trackLock");
        //zipTieSweeper = hardwareMap.servo.get("zipTieSweeper");
        ColorSense = hardwareMap.colorSensor.get("color");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        liftCheck = hardwareMap.touchSensor.get("liftCheck");
        servoClimberDumper.setPosition(1.0);
        //WHATEVER WE ARE GOING TO CALL THIS THING = hardwareMap.opticalDistanceSensor.get("STUFFANDTHINGS");
        nearMountainFlag = nearMountainSwitch.getState();
        //set up motors
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.FORWARD);
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        trackLifter.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        climberTriggerLeft.setPosition(0.5);
        climberTriggerRight.setPosition(0.5);
        armLock.setPosition(0.5);
        servoDist.setPosition(0.5);
        trackLock.setPosition(0.8);
    }

    @Override
    public void loop() {
        double rightDrive;
        double leftDrive;

        rightDrive = -gamepad1.right_stick_y;
        leftDrive = -gamepad1.left_stick_y;
        if (gamepad1.dpad_up) {
            rightDrive = 0.1;
            leftDrive = 0.1;
        } else if (gamepad1.dpad_down) {
            rightDrive = -0.1;
            leftDrive = -0.1;
        } else if (gamepad1.dpad_right) {
            rightDrive = -0.1;
            leftDrive = 0.1;
        } else if (gamepad1.dpad_left) {
            rightDrive = 0.1;
            leftDrive = -0.1;
        }
        motorRight.setPower(rightDrive);
        motorLeft.setPower(leftDrive);

        if (gamepad1.right_bumper) { // up
            if (liftCheck.isPressed()) {
                trackLifter.setPower(0.0);
            } else {
                trackLifter.setPower(0.5);
            }
        } else if (gamepad1.right_trigger > 0.1) { // down
            trackLifter.setPower(-0.5 * gamepad1.right_trigger);
        } else {
            trackLifter.setPower(-0.0);
        }
        if (gamepad1.left_bumper) {
            cowCatcher.setPosition(0.75);
//            cowCatcherUp = true;
        } else if (gamepad1.left_trigger > 0.5) {
//            cowCatcherUp = false;
            cowCatcher.setPosition(0.2);
//        } else if (!cowCatcherUp) {
//            cowCatcher.setPosition(0.2);
        }

        armExtend.setPower(-gamepad2.left_stick_y);

        if (gamepad2.left_bumper) {
            climberTriggerLeft.setPosition(1.0);
        } else if (gamepad2.left_trigger > 0.5) {
            climberTriggerLeft.setPosition(0.0);
        } else {
            climberTriggerLeft.setPosition(0.5);
        }

        if (gamepad2.right_bumper) {
            climberTriggerRight.setPosition(0.0);
        } else if (gamepad2.right_trigger > 0.5) {
            climberTriggerRight.setPosition(1.0);
        } else {
            climberTriggerRight.setPosition(0.5);
        }

        if (gamepad2.a) {
            servoClimberDumper.setPosition(0.1);
        } else {
           servoClimberDumper.setPosition(1.0);
        }
//        if (gamepad2.b) {
//            climberDoor.setPosition(0.5);
//        } else {
//            climberDoor.setPosition(0.2);
//        }
        if (gamepad2.y) {
            //zipTieSweeper.setPosition(.75);
        }
        if (gamepad2.start) {
            armLock.setPosition(0.3);
        }
        if (gamepad2.x) {
            armLock.setPosition(0.5);
        }
        if (gamepad2.dpad_up) {
            trackLock.setPosition(0.4);
        }
        if (gamepad2.dpad_down) {
            trackLock.setPosition(0.8);
        }

        telemetry.addData("trackLifter", Integer.toString(trackLifter.getCurrentPosition()));
        telemetry.addData("liftCheck", liftCheck.isPressed());
        telemetry.addData("ENCLeft", Integer.toString(motorLeft.getCurrentPosition()));
        telemetry.addData("ENCRight", Integer.toString(motorRight.getCurrentPosition()));
        telemetry.addData("trigger", gamepad1.right_trigger);
    }
}