package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

/**
 * Created by Robotics on 9/29/2015.
 */
public class CCHS5256TeleOp extends OpMode {

   final static double bpusher_MIN_RANGE = 0.20;
    final static double bpusher_MAX_RANGE = 0.80;
    final static double cdumper_MIN_RANGE = 0.00;
    final static double cdumper_MAX_RANGE = 1.00;

    double beaconPinionPosition;
    double beaconPusherPosition;
    double climberDumperPosition;

    // DcMotorControllers
    DcMotorController driveTrainController;
   //  DcMotorController hangingController;
    // DcMotors
    DcMotor leftDrive;
    DcMotor rightDrive;
   //  DcMotor chinUp;
    // ServoControllers
    ServoController beaconController;
   //  ServoController alignmentController;
    // Servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoUltraSense;
    Servo sweeper;
//    Servo servoClimberDumper;
   //  Servo leftOmniPinion;
   //  Servo rightOmniPinion;
    //sensors
    ColorSensor beaconColorSense;
//    ColorSensor floorColorSense;
//    OpticalDistanceSensor leftWheelAlignment;
//    OpticalDistanceSensor rightWheelAlignment;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;
    double lowDist;
    double medDist;
   //  TouchSensor beaconPinionAlignment;
   //  TouchSensor beaconPinionStop;
   //  TouchSensor leftWheelStop;
   //  TouchSensor rightWheelStop;
    // State Machine Options
    // Delay Settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    /**
     * Constructor
     */
    public CCHS5256TeleOp() {
    }
    
      void moveStraightWithGyro(double speed) {
         int preMoveHeading 

         if (gyroSense.getHeading == (preMoveHeading - 1) || gyroSense.getHeading == preMoveHeading || gyroSense.heading == (preMoveHeading + 1) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
         } else if (gyroSense.getHeading > (preMoveHeading + 1)) {
            leftDrive.setPower(speed);
            rightDrive.setPower(0.0);
         } else if (gyroSense.getHeading < (preMoveHeading - 1)) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(speed);
         }
    }
    
      void sweep(double speed) {
         if (speed == 0) {
            sweeper.setPosition(0.5);
         } else if (speed < 0 || speed > 0) {
            sweeper.setPostion((speed/2) + 0.5);
         }
      }

    @Override
    public void init() {
        // DcMotorControllers
        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
      //  hangingController = hardwareMap.dcMotorController.get("hangCtlr");
        // DcMotors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
      //  chinUp = hardwareMap.dcMotor.get("chinUp");
        // Servo Controllers
        beaconController = hardwareMap.servoController.get("beaconCtlr");
      //  alignmentController = hardwareMap.servoController.get("alignCtlr");
        // Servos
        servoBeaconPinion = hardwareMap.servo.get("beaconPinion");
        servoBeaconPusher = hardwareMap.servo.get("beaconPusher");
//        servoClimberDumper = hardwareMap.servo.get("climberDumper");
        servoUltraSense = hardwareMap.servo.get("servoUltra");
      //  leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
      //  rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        // Sensors
        beaconColorSense = hardwareMap.colorSensor.get("bColorSense");
        beaconColorSense.enableLed(false);
//        floorColorSense = hardwareMap.colorSensor.get("fColorSense");
//        floorColorSense.enableLed(true);
//        leftWheelAlignment = hardwareMap.opticalDistanceSensor.get("lWAlign");
//        rightWheelAlignment = hardwareMap.opticalDistanceSensor.get("rWAlign");
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
      //  beaconPinionAlignment = hardwareMap.touchSensor.get("bPALign");
      //  beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
      //  leftWheelStop = hardwareMap.touchSensor.get("lWStop");
      //  rightWheelStop = hardwareMap.touchSensor.get("rWStop");
        //motor configurations
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        // state machine settings
        // servo positions
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
        
        if (gamepad1.a) {
           if (ultraSense.getUltrasonicLevel <= lowDist) {
              moveStraightWithGyro(0.5);
           } else {
              moveStraightWithGyro(0.0)
           }
        } else if (gamepad1.b) {
           moveStraightWithGyro(-0.5);
        }

          if (gamepad1.x) {
           if (ultraSense.getUltrasonicLevel <= medDist) {
              moveStraightWithGyro(0.5);
           } else {
              moveStraightWithGyro(0.0)
           }
        } else if (gamepad1.y) {
           moveStraightWithGyro(-0.5);
        } 

            // update the position of the arm.
   //  if (gamepad2.a) {
   //     // if the A button is pushed on gamepad1, increment the position of
   //     // the arm servo.
   //     if(servoBeaconPusher.getPosition()< 0.5) {
   //          beaconPusherPosition = 1.0;
   //     }
   //     if(servoBeaconPusher.getPosition()>= 0.5) {
   //          beaconPusherPosition = 0.0;
   //     }
   //  }

        // clip the position values so that they never exceed their allowed range.
        beaconPusherPosition = Range.clip(beaconPinionPosition, bpusher_MIN_RANGE, bpusher_MAX_RANGE);
        
        // write position values to the wrist and claw servo
        servoBeaconPusher.setPosition(beaconPusherPosition);

        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("right trigger", gamepad1.right_trigger);
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
        // scale input * 4/3 (top speed = 1.0)
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
