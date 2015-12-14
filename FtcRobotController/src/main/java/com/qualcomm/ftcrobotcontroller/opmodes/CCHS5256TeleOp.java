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

    final static double bpinion_MIN_RANGE  = 0.20;
    //not sure, we are using a continuous
    final static double bpinnion_MAX_RANGE  = 0.90;
    //not sure, we are using a continuous
    final static double bpusher_MIN_RANGE  = 0.20;
    final static double bpusher_MAX_RANGE  = 0.80;
    final static double cdumper_MIN_RANGE  = 0.00;
    final static double cdumber_MAX_RANGE  = 1.00;
    final static double creleaser_MIN_RANGE  = 0.00;
    final static double creleaser_MAX_RANGE  = 1.00;

    // position of the arm servo.
    double beaconPinionPosition;

    // amount to change the arm servo position.
    double beaconPinionDelta = 0.1;

    // position of the claw servo
    double beaconPusherPosition;

    // amount to change the claw servo position by
    double beaconPusherDelta = 0.1;

    // position of the arm servo.
    double climberDumperPosition;

    // amount to change the arm servo position.
    double climberDumperDelta = 0.1;

    // position of the claw servo
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
Servo servoClimberDumper;
Servo servoClimberReleaser;
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
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1 = hardwareMap.dcMotor.get("right_drive");
//        armController1A = hardwareMap.dcMotor.get("arm_1A");
//        armController1B = hardwareMap.dcMotor.get("arm_1B");
//        armController2A = hardwareMap.dcMotor.get("arm_2A");
//        armController2B = hardwareMap.dcMotor.get("arm_2B");
//        servoController1 = hardwareMap.servoController.get("servo_controller_1");
        servoBeaconPinion = hardwareMap.servo.get("beacon_pinion");
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoClimberReleaser = hardwareMap.servo.get("climber_releaser");
//        servo1E = hardwareMap.servo.get("servo1E");
//        servo1F = hardwareMap.servo.get("servo1F");
//        leftDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        leftDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

       beaconPusherPosition = 0.20;
       climberDumperPosition = 0.00;
       climberReleaserPosition = 0.00;

    }
@Override
public void loop() {

float left = gamepad1.left_stick_y;
float right = gamepad1.right_stick_y;

left = Range.clip(left, -1, 1);
right = Range.clip(right, -1, 1);

left = (float)scaleInput(left);
right = (float)scaleInput(right);

left = (float)medium(left);
right = (float)medium(right);

// if (gamepad1.righttrigger) {
//     left = (float)fast(left);
//     right = (float)fast(right);
// }else if (gamepad1.lefttrigger) {
//     left = (float)slow(left);
//     right = (float)slow(right);
// }else {
//     left = (float)medium(left);
//     right = (float)medium(right)
// }

leftDrive1.setPower(left);
leftDrive2.setPower(left);
rightDrive1.setPower(right);
rightDrive2.setPower(right);

    // update the position of the arm.
    if (gamepad2.a) {
        // if the A button is pushed on gamepad1, increment the position of
        // the arm servo.
        if(servoBeaconPusher.getPosition()< 0.3) {
            beaconPusherPosition -= beaconPusherDelta;
        }
        if(servoBeaconPusher.getPosition()> 0.7) {
            beaconPusherPosition += beaconPusherDelta;
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
//    // write position values to the wrist and claw servo
//    arm.setPosition(armPosition);
//    claw.setPosition(clawPosition);


//telemetry.addData("enc_left_1", (float) leftDrive.getMotorCurrentPosition(1));
//telemetry.addData("enc_left_2", (float) leftDrive.getMotorCurrentPosition(2));
//telemetry.addData("enc_right_1", (float) rightDrive.getMotorCurrentPosition(1));
//telemetry.addData("enc_right_2", (float) rightDrive.getMotorCurrentPosition(2));
        }

@Override
public void stop() {

        }

    // double scaleInput(double dVal)  {
    //     double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
    //             0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

    //     // get the corresponding index for the scaleInput array.
    //     int index = (int) (dVal * 16.0);
    //     if (index < 0) {
    //         index = -index;
    //     } else if (index > 16) {
    //         index = 16;
    //     }

    //     double dScale = 0.0;
    //     if (dVal < 0) {
    //         dScale = -scaleArray[index];
    //     } else {
    //         dScale = scaleArray[index];
    //     }

    //     return dScale;
    // }
    
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.04, 0.07, 0.10, 0.13, 0.16, 0.19, 0.23,
                0.28, 0.34, 0.41, 0.49, 0.58, 0.68, 0.72, 0.75, 0.75 };

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
    
    // double fast(double dval) {
    // // scale input * 1.3333333333 (top speed = 1.0)    
    // }
    // double medium(double dval) {
    // // scale input * 1.0 (top speed = 0.75)  
    // }
    // double slow(double dval) {
    // // scale input * 0.5 (top speed = .375)    
    // }

}
