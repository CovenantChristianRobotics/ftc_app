package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
public class CCHS5256TeleOpOLD extends OpMode {

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
    // DcMotors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor endGameLights;
    DcMotor chinUp;
    // Servos
//    Servo servoBeaconPusher;
    Servo servoUltraSense;
//    Servo sweeper;
//    boolean sweepOn;
//    Servo servoClimberDumper;
     Servo leftOmniPinion;
     Servo rightOmniPinion;
    Servo armLock;
    Servo leftPlow;
    Servo rightPlow;
    //sensors
//    ColorSensor beaconColorSense;
//    ColorSensor floorColorSense;
//    OpticalDistanceSensor wheelAlignment;
    GyroSensor gyroSense;
//    boolean goStraightWithGyro;
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
    public CCHS5256TeleOpOLD() {
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
        // DcMotors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        chinUp = hardwareMap.dcMotor.get("chinUp");
        // Servos
//        servoBeaconPusher = hardwareMap.servo.get("beaconPusher");
//        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
//        servoUltraSense = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        armLock = hardwareMap.servo.get("armLock");
        armLock.setPosition(0.5);
        leftPlow = hardwareMap.servo.get("lP");
        rightPlow = hardwareMap.servo.get("rP");
        rightPlow.setDirection(Servo.Direction.REVERSE);
        // Sensors
//        beaconColorSense = hardwareMap.colorSensor.get("bColorSense");
//        beaconColorSense.enableLed(false);
//        floorColorSense = hardwareMap.colorSensor.get("fColorSense");
//        floorColorSense.enableLed(true);
//        wheelAlignment = hardwareMap.opticalDistanceSensor.get("wAlign");
//        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
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
        chinUp.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        // state machine settings
        currentControl = ledControl.PREMATCH;
        // servo positions
//        servoBeaconPusher.setPosition(0.5);
        endGameTime = new ElapsedTime();
        endGameLights.setPower(0.0);
//        goStraightWithGyro = true;
//        gyroSense.calibrate();
//        while (gyroSense.isCalibrating()) {
//        }
    }

    @Override
    public void loop() {

        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double hang = gamepad1.left_stick_x;
        float omniWheels = gamepad2.left_stick_y;
        float rightStickPos = -gamepad1.right_stick_y;
        float rightStickNeg = -gamepad1.right_stick_y;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        hang = Range.clip(hang, -1, 1);
        omniWheels = Range.clip(omniWheels, -1, 1);

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
        
        if (gamepad1.right_trigger > 0.5) {
            left = (float) fast(left);
            right = (float) fast(right);
        } else if (gamepad1.left_trigger > 0.5) {
            left = (float) slow(left);
            right = (float) slow(right);
        } else if (gamepad1.dpad_up) {
            left = rightStickPos * -1;
            right = rightStickPos * -1;
        } else if (gamepad1.dpad_down) {
            left = rightStickNeg * -1;
            right = rightStickNeg * -1;
        } else if (gamepad1.dpad_right) {
            left = -0.1;
            right = 0.1;
            if(gamepad1.right_trigger > 0.5) {
                left = (double) fast(left);
                right = (double) fast(right);
            } else if (gamepad1.left_trigger > 0.5) {
                left = (double) slow(left);
                right = (double) slow(right);
            } else {
                left = left;
                right = right;
            }
        } else if (gamepad1.dpad_left) {
            left = 0.1;
            right = -0.1;
            if(gamepad1.right_trigger > 0.5) {
                left = (double) fast(left);
                right = (double) fast(right);
            } else if (gamepad1.left_trigger > 0.5) {
                left = (float) slow(left);
                right = (float) slow(right);
            }
        } else if (gamepad1.y) {
            if (gamepad2.dpad_up) {
                left = -0.1;
                right = -0.1;
            } else if (gamepad2.dpad_down) {
                left = 0.1;
                right = 0.1;
            }
        }
        else {
            left = (float) medium(left);
            right = (float) medium(right);
             // left = (float) -0.05;   // for calibrating autonomous distances only
             // right = (float) +0.05;   // for calibrating autonomous distances only
        }
        
        if (gamepad1.y) {
            chinUp.setPower(hang);
        } else {
            chinUp.setPower(0.0);
        }

        if (gamepad1.x) {
            leftDrive.setPower(0.05);
            rightDrive.setPower(0.05);
        }

        if (gamepad2.a) {
            armLock.setPosition(0.2292121569);
        } else if (gamepad2.b) {
            armLock.setPosition(0.5);
        }


//        if (gamepad2.dpad_up) {
//            leftOmniPinion.setPosition(0.75);
//            rightOmniPinion.setPosition(0.25);
//        } else if (gamepad2.dpad_down) {
//            leftOmniPinion.setPosition(0.25);
//            rightOmniPinion.setPosition(0.75);
//        } else {
//            leftOmniPinion.setPosition(0.5);
//            rightOmniPinion.setPosition(0.5);
//        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);

        if (gamepad2.dpad_left) {
            leftOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
        } else if (gamepad2.dpad_right) {
            rightOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
        } else {
            rightOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
            leftOmniPinion.setPosition(((gamepad2.right_stick_y / 2) + 0.5));
        }

        if (gamepad2.y) {
            leftPlow.setPosition(Range.clip((leftPlow.getPosition() + 0.01), 0.0, 1.0));
            rightPlow.setPosition(Range.clip((rightPlow.getPosition() + 0.01), 0.0, 1.0));
        } else if (gamepad2.x) {
            leftPlow.setPosition(Range.clip((leftPlow.getPosition() - 0.01), 0.0, 1.0));
            rightPlow.setPosition(Range.clip((rightPlow.getPosition() - 0.01), 0.0, 1.0));
        }
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
        //
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
                moveDelayTime = 325;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BLINKOFF;
                break;

            case BLINKOFF:
                endGameLights.setPower(0.0);
                moveDelayTime = 200;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BlINKON;
                break;
        }

        telemetry.addData("LED", currentControl.toString());
        telemetry.addData("Elapsed Time", endGameTime.time());
        telemetry.addData("l plow", leftPlow.getPosition());
        telemetry.addData("r plow", rightPlow.getPosition());
        telemetry.addData("enc right", rightDrive.getCurrentPosition());
        telemetry.addData("enc left", leftDrive.getCurrentPosition());
        telemetry.addData("arm lock", armLock.getPosition());
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
