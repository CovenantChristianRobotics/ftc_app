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
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cchsrobochargers on 12/10/15.
 */
public class FastTeleOp extends OpMode {

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
    double climberTriggerLeftPos = 0.9;
    double climberTriggerRightPos = 0.1;

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
        servoClimberDumper.setPosition(0.0);
        //WHATEVER WE ARE GOING TO CALL THIS THING = hardwareMap.opticalDistanceSensor.get("STUFFANDTHINGS");
        nearMountainFlag = nearMountainSwitch.getState();
        //set up motors
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        trackLifter.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        cowCatcher.setPosition(0.1);
        climberTriggerLeft.setPosition(0.8);
        climberTriggerRight.setPosition(0.0);
        armLock.setPosition(0.5);
        servoDist.setPosition(0.5);
        trackLock.setPosition(0.45);
    }

    @Override
    public void loop() {
        double rightDrive;
        double leftDrive;

        rightDrive = -gamepad1.right_stick_y;
        leftDrive = -gamepad1.left_stick_y;
        if (gamepad1.dpad_up) {
            rightDrive = 0.3;
            leftDrive = 0.3;
        } else if (gamepad1.dpad_down) {
            rightDrive = -0.3;
            leftDrive = -0.3;
        } else if (gamepad1.dpad_right) {
            rightDrive = -0.3;
            leftDrive = 0.3;
        } else if (gamepad1.dpad_left) {
            rightDrive = 0.3;
            leftDrive = -0.3;
        }
        motorRight.setPower(rightDrive);
        motorLeft.setPower(leftDrive);

        if (gamepad1.a) {
            servoClimberDumper.setPosition(1.0);
        } else {
            servoClimberDumper.setPosition(0.0);
        }

        telemetry.addData("trackLifter", Integer.toString(trackLifter.getCurrentPosition()));
        telemetry.addData("liftCheck", liftCheck.isPressed());
        telemetry.addData("ENCLeft", Integer.toString(motorLeft.getCurrentPosition()));
        telemetry.addData("ENCRight", Integer.toString(motorRight.getCurrentPosition()));
        telemetry.addData("trigger", gamepad1.right_trigger);
    }
}