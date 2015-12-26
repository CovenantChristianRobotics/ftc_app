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
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
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
        //servos
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
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
    }

    @Override
    public void loop() {
        //getting the gamepad values on the y axis
        float right = gamepad1.left_stick_y;
        float left = gamepad1.right_stick_y;

        motorRight.setPower(right);
        motorLeft.setPower(left);
        if (gamepad2.a) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            trackLifter.setPower(0.0);
            trackLifter.setPowerFloat();
        } else if (gamepad2.right_bumper) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            trackLifter.setTargetPosition(trackLifterUp);
            trackLifter.setPower(0.1);
        } else if (gamepad2.right_trigger > 0.5) {
            trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            trackLifter.setTargetPosition(trackLifterUp + 1220 / 4);
            trackLifter.setPower(0.1);
        } else if (liftCheck.getLightDetected() > 0.3) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }
        telemetry.addData("trackLifter", (float)trackLifter.getCurrentPosition());
        telemetry.addData("liftCheck", liftCheck.getLightDetected());
        telemetry.addData("ENCLeft", (float)motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float)motorRight.getCurrentPosition());
    }
}