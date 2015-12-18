package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 9/29/2015.
 */
public class CCHS5256TeleOp extends OpMode {


    final static double bpusher_MIN_RANGE = 0.20;
    final static double bpusher_MAX_RANGE = 0.80;
    final static double cdumper_MIN_RANGE = 0.00;
    final static double cdumper_MAX_RANGE = 1.00;


    // position of the arm servo.
    double beaconPinionPosition;

    // position of the claw servo
    double beaconPusherPosition;

    // amount to change the claw servo position by
    double beaconPusherDelta = 0.1;

    // position of the arm servo.
    double climberDumperPosition;

    // amount to change the arm servo position.
    //double climberDumperDelta = 0.1;

    //position of the claw servo
    double climberReleaserPosition;

    // amount to change the claw servo position by
    double climberReleaserDelta = 0.1;


    DcMotorController driveTrainController;
    // DcMotorController rightDrive;
//DcMotorController armController1;
//DcMotorController armController2;
    DcMotor leftDrive;
    DcMotor rightDrive;
    //DcMotor armController1A;
//DcMotor armController1B;
//DcMotor armController2A;
//DcMotor armController2B;
//ServoController servoController1;
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
//    Servo servoClimberDumper;
//    Servo servoClimberReleaser;
//Servo servo1E;
//Servo servo1F;

    /**
     * Constructor
     */
    public CCHS5256TeleOp() {

    }

    @Override
    public void init() {


        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
//        armController1 = hardwareMap.dcMotorController.get("arm_controller_1");
//        armController2 = hardwareMap.dcMotorController.get("arm_controller_2");
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//        armController1A = hardwareMap.dcMotor.get("arm_1A");
//        armController1B = hardwareMap.dcMotor.get("arm_1B");
//        armController2A = hardwareMap.dcMotor.get("arm_2A");
//        armController2B = hardwareMap.dcMotor.get("arm_2B");
//        servoController1 = hardwareMap.servoController.get("servo_controller_1");
        servoBeaconPinion = hardwareMap.servo.get("beacon_pinion");
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
//        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
//        servo1E = hardwareMap.servo.get("servo1E");
//        servo1F = hardwareMap.servo.get("servo1F");
//        leftDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        leftDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

        beaconPusherPosition = 0.5;

    }

    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float beaconPinion = gamepad2.right_stick_y;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        beaconPinion = Range.clip(beaconPinion, -1, 1);

        if (gamepad1.right_trigger > 0) {
            left = (float) fast(left);
            right = (float) fast(right);
        } else if (gamepad1.left_trigger > 0) {
            left = (float) slow(left);
            right = (float) slow(right);
        } else {
            left = (float) medium(left);
            right = (float) medium(right);
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
        servoBeaconPinion.setPosition(((beaconPinion / 2) + 0.5));

            // update the position of the arm.
    if (gamepad2.a) {
        // if the A button is pushed on gamepad1, increment the position of
        // the arm servo.
        if(servoBeaconPusher.getPosition()< 0.5) {
            beaconPusherPosition = 1.0;
        }
        if(servoBeaconPusher.getPosition()>= 0.5) {
            beaconPusherPosition = 0.0;
        }
    }



//    if (gamepad1.y) {
//        // if the Y button is pushed on gamepad1, decrease the position of
//        // the arm servo.
//        armPosition -= armDelta;
//    }
//
//    // update the position of the claw
//    if (gamepad1.x) {
//        clawPosition += clawDelta;
//    }
//
//    if (gamepad1.b) {
//        clawPosition -= clawDelta;
//    }
//
//    // clip the position values so that they never exceed their allowed range.
//    armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
//    clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//
        beaconPusherPosition = Range.clip(beaconPinionPosition, bpusher_MIN_RANGE, bpusher_MAX_RANGE);
//    // write position values to the wrist and claw servo
servoBeaconPusher.setPosition(beaconPusherPosition);


//telemetry.addData("enc_left_1", (float) leftDrive.getMotorCurrentPosition(1));
//telemetry.addData("enc_left_2", (float) leftDrive.getMotorCurrentPosition(2));
//telemetry.addData("enc_right_1", (float) rightDrive.getMotorCurrentPosition(1));
//telemetry.addData("enc_right_2", (float) rightDrive.getMotorCurrentPosition(2));
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("Pinion", servoBeaconPinion.getPosition());
        telemetry.addData("Pusher", servoBeaconPusher.getPosition());
        }

        @Override
        public void stop () {
        }


    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.04, 0.07, 0.10, 0.13, 0.16, 0.19, 0.23,
                0.28, 0.34, 0.41, 0.49, 0.58, 0.68, 0.72, 0.75, 0.75};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    double fast(double jVal) {
        // scale input * 1.3333333333 (top speed = 1.0)
        return ((4 / 3) * scaleInput(jVal));
    }

    double medium(double jVal) {
        // scale input * 1.0 (top speed = 0.75)
        return scaleInput(jVal);
    }

    double slow(double jVal) {
        // scale input * 0.5 (top speed = .375)
        return (0.5 * scaleInput(jVal));
    }

}

