package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

/**
 * Created by Robotics on 9/29/2015.
 */
public class CCHS5256TeleOp extends OpMode {

    enum ledControl {
        PREMATCH, START, ON, ENDGAME, BLINKOFF, BlINKON, DELAYSETTINGS, DELAY
    }

//    final static double bpusher_MIN_RANGE = 0.20;
//    final static double bpusher_MAX_RANGE = 0.80;
//    final static double cdumper_MIN_RANGE = 0.00;
//    final static double cdumper_MAX_RANGE = 1.00;
//
//    double beaconPinionPosition;
//    double beaconPusherPosition;
//    double climberDumperPosition;
//
//    // DcMotorControllers
//    DcMotorController driveTrainController;
//     DcMotorController hangingController;
    // DcMotors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor endGameLights;
    DcMotor chinUp;
    // ServoControllers
//    ServoController beaconController;
   //  ServoController alignmentController;
    // Servos
//    Servo servoBeaconPinion;
//    Servo servoBeaconPusher;
//    Servo servoUltraSense;
//    Servo sweeper;
//    boolean sweepOn;
//    Servo servoClimberDumper;
     Servo leftOmniPinion;
     Servo rightOmniPinion;
    //sensors
//    ColorSensor beaconColorSense;
//    ColorSensor floorColorSense;
//    OpticalDistanceSensor wheelAlignment;
    GyroSensor gyroSense;
//    boolean goStraightWithGyro;
//    UltrasonicSensor fUltraSense;
//    UltrasonicSensor bUltraSense;
//    double lowDist;
//    double medDist;
   //  TouchSensor beaconPinionAlignment;
   //  TouchSensor beaconPinionStop;
//     TouchSensor leftWheelStop;
//     TouchSensor rightWheelStop;
    // State Machine Options
    ledControl currentControl;
    ledControl nextControl;
    ElapsedTime endGameTime;
    // Delay Settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    /**
     * Constructor
     */
    public CCHS5256TeleOp() {
    }
    
//      void moveStraightWithGyro(double speed) {
//         int centerDist;
//         int preMoveHeading;
//         int currentHeading;
//
//         preMoveHeading = gyroSense.getHeading();
//         centerDist = 180 - preMoveHeading;
//
//         while (goStraightWithGyro == true) {
//             currentHeading = gyroSense.getHeading();
//
//             if (currentHeading + centerDist > 359) {
//                 currentHeading = currentHeading - 360;
//             } else if ( currentHeading + centerDist < 0) {
//                 currentHeading = currentHeading + 360;
//             }
//
//             if (currentHeading == preMoveHeading) {
//                 leftDrive.setPower(speed);
//                 rightDrive.setPower(speed);
//             } else if (currentHeading != preMoveHeading){
//                 if ((currentHeading + centerDist) > (preMoveHeading + centerDist)) {
//                     leftDrive.setPower(speed * 0.2);
//                     rightDrive.setPower(-speed * 0.2);
//                 } else if ((currentHeading + centerDist) < (preMoveHeading + centerDist)) {
//                     leftDrive.setPower(-speed * 0.2);
//                     rightDrive.setPower(speed * 0.2);
//                 }
//             }
//
//         }
//
//    }
    
//      void sweep(double speed) {
//         if (speed == 0) {
//            sweeper.setPosition(0.5);
//            sweepOn = false;
//         } else if (speed < 0 || speed > 0) {
//            sweeper.setPosition((speed/2) + 0.5);
//            sweepOn = true;
//         }
//      }
    /**
     * speed is between -1 and +1
     *
     * @param lOmnipinionSpeed
     */
//    void moveLeftOmnipinion(double lOmnipinionSpeed) {
//        double FomniPinionSpeed = ((lOmnipinionSpeed / 2) + 0.5);
//        servoBeaconPinion.setPosition(FomniPinionSpeed);
//    }

    /**
//     * @param rOmnipinionSpeed speed is between -1 and +1
     */
//    void moveRightOmnipinion(double rOmnipinionSpeed) {
//        double FomniPinionSpeed = ((rOmnipinionSpeed / 2) + 0.5);
//        servoBeaconPinion.setPosition(FomniPinionSpeed);
//    }

    @Override
    public void init() {
        // DcMotorControllers
//        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
//        hangingController = hardwareMap.dcMotorController.get("hangCtlr");
        // DcMotors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        chinUp = hardwareMap.dcMotor.get("chinUp");
        // Servo Controllers
//        beaconController = hardwareMap.servoController.get("beaconCtlr");
//        alignmentController = hardwareMap.servoController.get("alignCtlr");
        // Servos
//        servoBeaconPinion = hardwareMap.servo.get("beaconPinion");
//        servoBeaconPusher = hardwareMap.servo.get("beaconPusher");
//        servoClimberDumper = hardwareMap.servo.get("climberDumper");
//        servoUltraSense = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        // Sensors
//        beaconColorSense = hardwareMap.colorSensor.get("bColorSense");
//        beaconColorSense.enableLed(false);
//        floorColorSense = hardwareMap.colorSensor.get("fColorSense");
//        floorColorSense.enableLed(true);
//        wheelAlignment = hardwareMap.opticalDistanceSensor.get("wAlign");
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
//        fUltraSense = hardwareMap.ultrasonicSensor.get("fUltraSense");
//        bUltraSense = hardwareMap.ultrasonicSensor.get("bUltraSense");
      //  beaconPinionAlignment = hardwareMap.touchSensor.get("bPALign");
      //  beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
//        leftWheelStop = hardwareMap.touchSensor.get("lWStop");
//        rightWheelStop = hardwareMap.touchSensor.get("rWStop");
        //motor configurations
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        // state machine settings
        currentControl = ledControl.PREMATCH;
        // servo positions
//        beaconPusherPosition = 0.5;
        endGameTime = new ElapsedTime();
        endGameLights.setPower(0.0);
//        goStraightWithGyro = true;
//        gyroSense.calibrate();
//        while (gyroSense.isCalibrating()) {
//        }
    }

    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float beaconPinion = gamepad2.right_stick_y;
        float rightStickPos = gamepad1.right_stick_y;
        float rightStickNeg = gamepad1.right_stick_y;

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
//            left = (float) -0.05;
//            right = (float) -0.05;
        }

        if (rightStickPos < 0 ) {
            rightStickPos = 0;
        } else if (rightStickPos >= 0) {
            rightStickPos = gamepad1.right_stick_y;
        }

        if (rightStickNeg > 0 ) {
            rightStickNeg = 0;
        } else if (rightStickNeg <= 0) {
            rightStickNeg = gamepad1.right_stick_y;
        }
//
        if (gamepad1.dpad_up) {
            // moveStraightWithGyro(rightStickPos);
            leftDrive.setPower(rightStickPos);
            rightDrive.setPower(rightStickPos);
        } else if (gamepad1.dpad_down) {
            // moveStraightWithGyro(rightStickNeg);
            leftDrive.setPower(rightStickNeg);
            rightDrive.setPower(rightStickNeg);
        }

//        if (gamepad2.dpad_up) {
//            if (leftWheelStop.isPressed() == false && rightWheelStop.isPressed() == false) {
//                moveLeftOmnipinion(0.5);
//                moveRightOmnipinion(0.5);
//            } else if (leftWheelStop.isPressed() == true || rightWheelStop.isPressed() == true) {
//                moveLeftOmnipinion(0.0);
//                moveRightOmnipinion(0.0);
//            } else if (wheelAlignment.getLightDetected() > 0.3) {
//                moveLeftOmnipinion(0.0);
//                moveRightOmnipinion(0.0);
//            }
//        }
//
//        if (gamepad2.dpad_down) {
//            if (leftWheelStop.isPressed() == false && rightWheelStop.isPressed() == false) {
//                moveLeftOmnipinion(-0.5);
//                moveRightOmnipinion(-0.5);
//            } else if (leftWheelStop.isPressed() == true || rightWheelStop.isPressed() == true) {
//                moveLeftOmnipinion(0.0);
//                moveRightOmnipinion(0.0);
//            } else if (wheelAlignment.getLightDetected() > 0.3) {
//                moveLeftOmnipinion(0.0);
//                moveRightOmnipinion(0.0);
//            }
//        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);
//        servoBeaconPinion.setPosition(((beaconPinion / 2) + 0.5));
//
//        if (gamepad1.a) {
//           if (bUltraSense.getUltrasonicLevel() <= lowDist) {
//              moveStraightWithGyro(0.5);
//           } else {
//              moveStraightWithGyro(0.0);
//           }
//        } else if (gamepad1.b) {
//           moveStraightWithGyro(-0.5);
//        }
//
//          if (gamepad1.x) {
//           if (bUltraSense.getUltrasonicLevel() <= medDist) {
//              moveStraightWithGyro(0.5);
//           } else {
//              moveStraightWithGyro(0.0);j3
//           }
//        } else if (gamepad1.y) {
//           moveStraightWithGyro(-0.5);
//        }

        

            // update the position of the arm.
//        if (gamepad2.a) {
//            beaconPusherPosition = 0.7;
//        } else if (gamepad2.b) {
//            beaconPusherPosition = 0.3;
//        }

        // write position values to the wrist and claw servo
//        servoBeaconPusher.setPosition(beaconPusherPosition);

        switch (currentControl) {
            case DELAYSETTINGS:
                now = new Date();
                delayUntil = now.getTime() + moveDelayTime;
                currentControl = ledControl.DELAY;
                break;

            case DELAY:
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentControl = nextControl;
                }
                break;

            case PREMATCH:
                currentControl = ledControl.START;
                break;

            case START:
                endGameTime.reset();
                endGameLights.setPower(1.0);
                currentControl = ledControl.ON;
                break;

            case ON:
                if (endGameTime.time() > 90.0) {
                    currentControl = ledControl.ENDGAME;
                } else {
                    currentControl = ledControl.ON;
                }
                break;

            case ENDGAME:
                endGameLights.setPower(0.0);
                currentControl = ledControl.BlINKON;
                break;

            case BlINKON:
                endGameLights.setPower(1.0);
                moveDelayTime = 500;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BLINKOFF;
                break;

            case BLINKOFF:
                endGameLights.setPower(0.0);
                moveDelayTime = 500;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BlINKON;
                break;
        }

        telemetry.addData("LED", currentControl.toString());
        telemetry.addData("Elapsed Time", endGameTime.time());
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
