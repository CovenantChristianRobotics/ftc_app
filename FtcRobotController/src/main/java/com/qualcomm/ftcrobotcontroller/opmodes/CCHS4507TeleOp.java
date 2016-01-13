package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by cchsrobochargers on 12/10/15.
 */
public class CCHS4507TeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    DcMotor armPivot;
    DcMotor armExtend;

    //servos
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    Servo climberTriggerLeft;
    Servo climberTriggerRight;
    ColorSensor ColorSense;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;
    OpticalDistanceSensor liftCheck;
    //switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    boolean nearMountainFlag = false;
    boolean redAllianceFlag = false;
    int trackLifterUp = 0;


    @Override
    public void init() {
        //Ints hardware
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");
        armPivot = hardwareMap.dcMotor.get("armPivot");
        armExtend = hardwareMap.dcMotor.get("armExtend");

        //servos
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
        climberTriggerLeft = hardwareMap.servo.get("trigLeft");
        climberTriggerRight = hardwareMap.servo.get("trigRight");
        ColorSense = hardwareMap.colorSensor.get("color");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        liftCheck = hardwareMap.opticalDistanceSensor.get("liftCheck");
        //WHATEVER WE ARE GOING TO CALL THIS THING = hardwareMap.opticalDistanceSensor.get("STUFFANDTHINGS");
        nearMountainFlag = nearMountainSwitch.getState();
        //set up motors
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        armPivot.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armExtend.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    @Override
    public void loop() {
        motorRight.setPower(gamepad1.left_stick_y);
        motorLeft.setPower(gamepad1.right_stick_y);

        if (gamepad1.y) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            trackLifter.setPower(0.0);
            trackLifter.setPowerFloat();
        } else if (gamepad1.right_bumper) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            trackLifter.setTargetPosition(trackLifterUp);
            trackLifter.setPower(0.1);
        } else if (gamepad1.right_trigger > 0.5) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            trackLifter.setTargetPosition(trackLifterUp + 1220 / 4);
            trackLifter.setPower(0.1);
        } else if (liftCheck.getLightDetected() > 0.3) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }

        armPivot.setPower(gamepad2.right_stick_x);
        armExtend.setPower(gamepad2.left_stick_y);

        if (gamepad2.left_bumper) {
            climberTriggerLeft.setPosition(0.0);
        } else {
            climberTriggerLeft.setPosition(1.0);
        }

        if (gamepad2.right_bumper) {
            climberTriggerRight.setPosition(0.0);
        } else {
            climberTriggerRight.setPosition(1.0);
        }

        telemetry.addData("trackLifter", (float)trackLifter.getCurrentPosition());
        telemetry.addData("liftCheck", liftCheck.getLightDetected());
        telemetry.addData("ENCLeft", (float)motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float)motorRight.getCurrentPosition());
    }
}