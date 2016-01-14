package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Date;

/**
 * Created by Robotics on 10/7/2015.
 */
public class CCHS5256Autonomous extends OpMode {
    //    enum MoveState {
//        DELAY, STARTSTRAIGHTMOVE, STRAIGHTMOVING, STARTENCODERTURN, ENCODERTURNMOVING, GYROTURN,
//        PRETURN, SERVOPINIONREDMOVE, SERVOPINIONNULLMOVE, SERVOPINIONBLUEMOVE, SERVOPUSHERMOVE,
//        SERVODUMPERMOVE, MOVEDELAY, FIRSTMOVE, TURNDIAGWITHGYRO, TURNDIAGWITHENCODERS, MOVEDIAG,
//        FINDWALL, TURNALONGWALLWITHGYRO, TURNALONGWALLWITHENCODERS, DRIVEALONGWALL, FINDBEACON,
//        ALIGNDUMPER, DUMPCLIMBERS, ALIGNPRESSER, PRESSBUTTON, PULLAHEADALONGWALL,
//        ROTATEFROMBEACONWITHGYRO, ROTATEFROMBEACONWITHENCODERS, MOVETORAMP, TURNTORAMPWITHGYRO,
//        TURNTORAMPWITHENCODERS, UPRAMP, DONE
//    }
    enum MoveState {
        DELAY, STARTMOVE, MOVING, STARTTURN, TURNING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG,
        FINDWALL, TURNALONGWALL, DRIVETOBEACON, FINDBEACON, CENTERBUCKET, DUMPTRUCK, ROTATEFROMBEACON,
        MOVETORAMP, TURNTORAMP, STOPATRAMP, UPRAMP, DONE, GETVALUES, CHOOSEFIRST, PUSHBUTTON, UNPUSHBUTTON, ALIGNPUSHER, ALIGNDUMPER,
        DUMPCLIMBERS
    }

    // maximum and minimum values to use when clipping the ranges
    final static double bpusher_MIN_RANGE = 0.20;
    final static double bpusher_MAX_RANGE = 0.80;
    final static double cdumper_MIN_RANGE = 0.00;
    final static double cdumber_MAX_RANGE = 1.00;

    // target values for servos
    double beaconPusherPosition;
    double climberDumperPosition;
    double beaconPinionPosition;

    //dc motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    //    DcMotor chinUp;
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoUltraSense;
    //    Servo leftOmniPinion;
//    Servo rightOmniPinion;
    //LED
//    LED endGameLights;
    //sensors
    ColorSensor beaconColorSense;
    //    ColorSensor floorColorSense;
//    OpticalDistanceSensor leftWheelAlignment;
//    OpticalDistanceSensor rightWheelAlignment;
    GyroSensor gyroSense;
    int preTurnHeading;
    int gyroError;
    int desiredHeading;
    UltrasonicSensor ultraSense;
    TouchSensor beaconPinionStop;
    //    TouchSensor leftWheelStop;
//    TouchSensor rightWheelStop;
    TouchSensor beaconPinionIn;
    TouchSensor beaconPinionOut;
    //Statemachine options
    MoveState currentMove;
    MoveState nextMove;
    MoveState turnMove;
    MoveState telemetryMove;
    double ifRedOnBeacon;
    double ifBlueOnBeacon;
    boolean lookingForFlag;
    boolean dumpedClimbers;
    boolean pushedButton;
    double speed;
    boolean movingForward;
    double fastSpeed;
    double slowSpeed;
    double turnSpeed;
    // Switches
    DigitalChannel nearMtnSwitch;
    DigitalChannel redBlueBeaconSwitch;
    DigitalChannel delayPotSwitch;
    //    DigitalChannel tileSwitch;
    AnalogInput delayPotentiometer;
    int nearMtn;
    int redBlue;
    int delay;
    long delayTime;
    boolean redAlliance = false;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    boolean sawBlueFlag;
    boolean sawRedFlag;
    int dumperCounter = 0;
    double dumperPosition = 0.9;
    boolean nearMountainFlag = false;
    //    int tile;
    ElapsedTime matchTime;

    //delay settings
    long delayUntil;
    long moveDelayTime;
    Date now;
    double countsPerDonut = 14161.0;    // Encoder counts per 360 degrees
    double countsPerMeter = 10439.0;    // Found this experimentally: Measured one meter, drove distance, read counts
    int dumperCounterThresh = 8;       // Doesn't let the dumper counter get above a certain number

    public CCHS5256Autonomous() {
    }

    // Figure out how far off we are at the end of the previous move so we can correct
    void moveTurn(double degrees, double targetSpeed) {
        int rightTarget;
        int leftTarget;

        // Figure out how far off we are at the end of the previous move so we can correct
        gyroError = desiredHeading - gyroSense.getHeading();
        if (gyroError > 180) {
            gyroError = 360 - gyroError;
        }
        if (gyroError < -180) {
            gyroError = 360 + gyroError;
        }

        desiredHeading = desiredHeading + (int) degrees;
        if (desiredHeading >= 360) {
            desiredHeading = desiredHeading - 360;
        }
        if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
        }
        speed = targetSpeed;
        leftTarget = leftDrive.getCurrentPosition() - degreesToCounts(degrees + gyroError);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + degreesToCounts(degrees + gyroError);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(targetSpeed);
        rightDrive.setPower(targetSpeed);
    }

    int degreesToCounts(double degrees) {
        return (int) (degrees * (countsPerDonut / 360.0));
    }

    void moveStraight(double distanceCM, double targetSpeed) {
        int rightTarget;
        int leftTarget;

        speed = targetSpeed;
        if (distanceCM > 0.0) {
            movingForward = true;
        } else {
            movingForward = false;
        }
        leftTarget = leftDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(targetSpeed);
        rightDrive.setPower(targetSpeed);
    }

    int centimetersToCounts(double centimeters) {
        return (int) (centimeters * (countsPerMeter / 100.0));
    }

//    int centimetersToCounts(double centimeters) {
//        return (int)(36.09 * centimeters);
//    }
//
//    double countsToCentimeters(int counts) {
//        return (int)((double)counts / 36.09);
//    }
//
//    int degreesToCounts(double degrees) {
//        return (int)((6077.0 / 360.0) * degrees);
//    }
//
//    void moveStraight(double distanceCM, double speed) {
//        int rightTarget;
//        int leftTarget;
//
//        leftTarget = leftDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
//        leftDrive.setTargetPosition(leftTarget);
//        rightTarget = rightDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
//        rightDrive.setTargetPosition(rightTarget);
//        leftDrive.setPower(speed);
//        rightDrive.setPower(speed);
//    }
//
//    /**
//     * if degree magnitude is negative, robot turns clockwise
//     *
//     * @param degrees
//     * @param speed
//     */
//    void moveTurn(double degrees, double speed) {
//        int rightTarget;
//        int leftTarget;
//
//        leftTarget = leftDrive.getCurrentPosition() - degreesToCounts(degrees) ;
//        leftDrive.setTargetPosition(leftTarget);
//        rightTarget = rightDrive.getCurrentPosition() + degreesToCounts(degrees);
//        rightDrive.setTargetPosition(rightTarget);
//        leftDrive.setPower(speed);
//        rightDrive.setPower(speed);
//    }
//
//    /**
//     * if degree magnitude is negative, robot turns clockwise
//     *
//     * @param degrees
//     */
//    void moveTurnWithGyro(int degrees, boolean reverseByColor) {
//        int turnTo;
//        int centerDist;
//        int currentHeading;
//        int difference;
//
//        turnTo = preTurnHeading + degrees;
//        if (turnTo > 359) {
//            turnTo = turnTo - 360;
//        } else if (turnTo < 0) {
//            turnTo = turnTo + 360;
//        }
//
//        centerDist = 180 - turnTo;
//        currentHeading = gyroSense.getHeading();
//
//        while (currentHeading != turnTo) {
//            difference = (turnTo + centerDist) - (currentHeading + centerDist);
//            currentHeading = gyroSense.getHeading();
//
//            if (reverseByColor == true) {
//                if (currentHeading + centerDist > turnTo + centerDist) {
//                    leftDrive.setPower((-(difference / 100) * 3) * redBlue);
//                    rightDrive.setPower(((difference / 100) * 3) * redBlue);
//                } else if (currentHeading + centerDist > turnTo + centerDist) {
//                    leftDrive.setPower(((difference / 100) * 3) * redBlue);
//                    rightDrive.setPower((-(difference / 100) * 3) * redBlue);
//                }
//            } else if (reverseByColor == false){
//                if (currentHeading + centerDist > turnTo + centerDist) {
//                    leftDrive.setPower(-(difference / 100) * 3);
//                    rightDrive.setPower((difference / 100) * 3);
//                } else if (currentHeading + centerDist > turnTo + centerDist) {
//                    leftDrive.setPower((difference / 100) * 3);
//                    rightDrive.setPower(-(difference / 100) * 3);
//                }
//
//            }
//        }
//
//        if (reverseByColor == true) {
//
//
//        } else if (reverseByColor == false){
//
//            }
//        }
//
//
//
//
//    void motorOn(double speed, boolean onoroff) {
//        if(onoroff = true){
//            leftDrive.setPower(speed);
//            rightDrive.setPower(speed);
//            leftDrive.setTargetPosition(1000000000);
//            rightDrive.setTargetPosition(1000000000);
//        }
//        if (onoroff = false) {
//            leftDrive.setPower(0.0);
//            rightDrive.setPower(0.0);
//        }
//    }
//
//    /**
//     * speed is between -1 and +1
//     *
//     * @param beaconPinionSpeed
//     */
//    void moveBeaconPinion(double beaconPinionSpeed) {
//        double FbeaconPinionSpeed = ((beaconPinionSpeed / 2) + 0.5);
//        servoBeaconPinion.setPosition(FbeaconPinionSpeed);
//    }
//
//    /**
//     * since beaconPresserPosition is clipped, values can be entered between 0.0 and 0.1
//     *
//     * @param beaconPresserPosition
//     */
//    void moveBeaconPress(double beaconPresserPosition) {
//        beaconPresserPosition = Range.clip(beaconPresserPosition, bpusher_MIN_RANGE, bpusher_MAX_RANGE);
//        servoBeaconPusher.setPosition(beaconPresserPosition);
//        beaconPusherPosition = beaconPresserPosition;
//    }
//
//    /**
//     * since climberDumperPosition is clipped, values can be entered between 0.0 and 0.1
//     *
//     * @param climberDumperPosition
//     */
//    void moveClimberDump(double climberDumperPosition) {
//        climberDumperPosition = Range.clip(climberDumperPosition, cdumper_MIN_RANGE, cdumber_MAX_RANGE);
//        servoClimberDumper.setPosition(climberDumperPosition);
//        climberDumperPosition = climberDumperPosition;
//    }
//
//    /**
//     * speed is between -1 and +1
//     *
//     * @param lOmnipinionSpeed
//     */
//    void moveLeftOmnipinion(double lOmnipinionSpeed) {
//        double FomniPinionSpeed = ((lOmnipinionSpeed / 2) + 0.5);
//        servoBeaconPinion.setPosition(FomniPinionSpeed);
//    }
//
//    /**
//     * speed is between -1 and +1
//     *
//     * @param rOmnipinionSpeed
//     */
//    void moveRightOmnipinion(double rOmnipinionSpeed) {
//        double FomniPinionSpeed = ((rOmnipinionSpeed / 2) + 0.5);
//        servoBeaconPinion.setPosition(FomniPinionSpeed);
//    }

    @Override
    public void init() {
        //dc Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
//        chinUp = hardwareMap.dcMotor.get("chinUp");
        //servos
        servoBeaconPinion = hardwareMap.servo.get("beaconPinion");
        servoBeaconPusher = hardwareMap.servo.get("beaconPusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoUltraSense = hardwareMap.servo.get("servoUltra");
//        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
//        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        //LED
//        endGameLights = hardwareMap.led.get("endGameLights");
//        endGameLights.enable(false);
        //sensors
        beaconColorSense = hardwareMap.colorSensor.get("bColorSense");
        beaconColorSense.enableLed(false);
//        floorColorSense = hardwareMap.colorSensor.get("fColorSense");
//        floorColorSense.enableLed(true);
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        gyroSense.calibrate();
        ultraSense = hardwareMap.ultrasonicSensor.get("UltraSense");
        beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
//        leftWheelStop = hardwareMap.touchSensor.get("lWStop");
//        rightWheelStop = hardwareMap.touchSensor.get("rWStop");
        // beaconPinionIn = hardwareMap.touchSensor.get("bPIn");
        // beaconPinionOut = hardwareMap.touchSensor.get("bPOut");
        //motor configurations
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //statemachine settings
        currentMove = MoveState.MOVEDELAY;
        nextMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        lookingForFlag = false;
        // switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
        delayPotSwitch = hardwareMap.digitalChannel.get("dSw");
        delayPotentiometer = hardwareMap.analogInput.get("dP");
        delayTime = (long) (delayPotentiometer.getValue() * (10000 / 1024));
        matchTime = new ElapsedTime();
        //servo positions
        servoBeaconPinion.setPosition(0.0);
        servoClimberDumper.setPosition(1.0);
        servoBeaconPusher.setPosition(0.3);
        servoUltraSense.setPosition(0.25);
        moveDelayTime = delayTime;
        dumpedClimbers = false;
        pushedButton = false;
        if (redBlueBeaconSwitch.getState()) { //red alliance
            redBlue = 1;
            servoUltraSense.setPosition(.75);
        } else { // blue alliance
            redBlue = -1;
            servoUltraSense.setPosition(.25);
        }
        if (redBlueBeaconSwitch.getState()) { //This is for when we're going to blue
            redAlliance = false;
            lookingForRedFlag = false;
            lookingForBlueFlag = true;
            servoUltraSense.setPosition(0.75);
        } else { //This is for red
            redAlliance = true;
            lookingForRedFlag = true;
            lookingForBlueFlag = false;
            servoUltraSense.setPosition(0.25);
        }
        // align color sensor
        // if (redAlliance) {
        //     while (beaconPinionIn.isPressed == false) {
        //         servoBeaconPinion.setPosition(1.0);
        //     }
        // } else {
        //     while (beaconPinionOut.isPressed == true) {
        //         servoBeaconPinion.setPosition(0.0);
        //     }
        // }
        //  while (!beaconPinionStop.isPressed()) {
        //      servoBeaconPinion.setPosition(0.5);
        //  }
        // // align omniwheels
        // while (!leftWheelStop.isPressed()) {
        //     moveLeftOmnipinion(0.5);
        // }
        // while (!rightWheelStop.isPressed()) {
        //     moveRightOmnipinion(0.5);
        // }
        while (gyroSense.isCalibrating()) {
        }
    }

    @Override
    public void loop() {
        double distanceToWall = 0.0;
        double distance = 0.0;
        if (gyroSense.isCalibrating()) {
            return;
        }

        switch (currentMove) {

            case STARTMOVE:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.MOVING;
                }
                break;

            case MOVING:
                gyroError = desiredHeading - gyroSense.getHeading();
                if (gyroError > 180) {
                    gyroError = 360 - gyroError;
                }
                if (gyroError < -180) {
                    gyroError = 360 + gyroError;
                }
                if (!movingForward) {
                    gyroError = -gyroError;
                }
                rightDrive.setPower(Range.clip(speed + (gyroError * 0.2), -1.0, 1.0));
                leftDrive.setPower(Range.clip(speed - (gyroError * 0.2), -1.0, 1.0));
                if (lookingForFlag && (beaconColorSense.red() >= 1)) {
                    rightDrive.setPower(0.0);
                    leftDrive.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (lookingForFlag && (beaconColorSense.blue() >= 1)) {
                    rightDrive.setPower(0.0);
                    leftDrive.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case STARTTURN:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.TURNING;
                }
                break;

            case TURNING:
                gyroError = gyroSense.getHeading() - desiredHeading;
                if (gyroError > 180) {
                    gyroError = 360 - gyroError;
                }
                if (gyroError < -180) {
                    gyroError = 360 + gyroError;
                }
                //                moveTurn(gyroError, speed);
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case MOVEDELAY:
                now = new Date();
                delayUntil = now.getTime() + moveDelayTime;
                currentMove = MoveState.DELAY;
                break;

            case DELAY:
                if (leftDrive.isBusy() || rightDrive.isBusy()) {
                    // If we aren't quite done moving, restart the delay
                    currentMove = MoveState.MOVEDELAY;
                } else {
                    now = new Date();
                    if (now.getTime() >= delayUntil) {
                        currentMove = nextMove;
                    }
                }
                break;

            case FIRSTMOVE:
                moveStraight(60.0, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = delay;
                break;

            case TURNDIAG:
                if (redAlliance) {
                    moveTurn(-45.0, turnSpeed);
                } else {
                    moveTurn(45.0, turnSpeed);
                }
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.MOVEDIAG;
                telemetryMove = MoveState.TURNDIAG;
                moveDelayTime = delay;
                break;

            case MOVEDIAG:
                moveStraight(209.0, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDWALL;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = delay;
                break;

            case FINDWALL:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 80.0)) {
                    if (redAlliance) {
                        moveStraight((distanceToWall - 18.0) * 1.414, slowSpeed);
                    } else {
                        moveStraight((distanceToWall - 34.0) * 1.414, slowSpeed);
                    }
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNALONGWALL;
                    telemetryMove = MoveState.FINDWALL;
                    moveDelayTime = delay;
                }
                break;

            case TURNALONGWALL:
                if (redAlliance) {
                    moveTurn(45.0, turnSpeed); //original is -45
                } else {
                    moveTurn(135.0, turnSpeed);
                }
                currentMove = MoveState.STARTTURN;
//                nextMove = MoveState.FINDBEACON;
                nextMove = MoveState.DRIVETOBEACON;
                telemetryMove = MoveState.TURNALONGWALL;
                moveDelayTime = delay;
                break;

            case DRIVETOBEACON:
                if (redAlliance) {
                    moveStraight(-90.0, fastSpeed);
                } else {
                    moveStraight(90.0, fastSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CHOOSEFIRST;
                telemetryMove = MoveState.DRIVETOBEACON;
                moveDelayTime = delay;
                break;

            case FINDBEACON:
                if (redAlliance) {
                    moveStraight(-90.0, slowSpeed);
                } else {
                    moveStraight(90.0, slowSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CENTERBUCKET;
                telemetryMove = MoveState.FINDBEACON;
                moveDelayTime = delay;
                break;

            case GETVALUES:
                ifBlueOnBeacon = beaconColorSense.blue();
                ifRedOnBeacon = beaconColorSense.red();
                break;

            case CHOOSEFIRST:
                if (redAlliance == true && ifRedOnBeacon >= 1.0) {
                    currentMove = MoveState.PUSHBUTTON;
                } else if (redAlliance == false && ifBlueOnBeacon >= 1.0) {
                    currentMove = MoveState.PUSHBUTTON;
                } else {
                    currentMove = MoveState.ALIGNDUMPER;
                }
                break;

            //  case ALIGNPUSHER:
            //     if (dumpedClimbers == false) {
            //         currentMove = MoveState.PUSHBUTTON;
            //     } else {
            //         if(redAlliance) {
            //             while (colorSense.red() <= 1.0) {
            //                 servoBeaconPinion.setPosition(1.0);
            //             }
            //         } else {
            //             while (colorSense.blue() <= 1.0) {
            //                 servoBeaconPinion.setPosition(1.0);
            //             }
            //         }
            //         currentMove = MoveState.PUSHBUTTON;
            //     }    

            case PUSHBUTTON:
                servoBeaconPusher.setPosition(0.7);
                moveDelayTime = 1000; // mSec - time for the servo to move
                currentMove = MoveState.MOVEDELAY;
                nextMove = MoveState.UNPUSHBUTTON;
                telemetryMove = MoveState.PUSHBUTTON;
                break;

            case UNPUSHBUTTON:
                servoBeaconPusher.setPosition(0.3);
                pushedButton = true;
                if (dumpedClimbers == true) {
                    currentMove = MoveState.ROTATEFROMBEACON;
                } else if (dumpedClimbers == false) {
                    currentMove = MoveState.ALIGNDUMPER;
                }
                telemetryMove = MoveState.UNPUSHBUTTON;
                break;

            case ALIGNDUMPER:
                if (beaconColorSense.red() >= 1.0 || beaconColorSense.blue() >= 1.0) {
                    servoBeaconPusher.setPosition(1.0);
                } else {
                    servoBeaconPusher.setPosition(0.5);
                    currentMove = MoveState.DUMPCLIMBERS;
                }
                telemetryMove = MoveState.ALIGNDUMPER;
                //  if (redAlliance) {
                //      while (colorSense.red() >= 1.0 || colorSense.blue() >=1.0) {
                //          servoBeaconPinion.setPosition(0.0);
                //      }
                //  } else {
                //      while (colorSense.red() >= 1.0 || colorSense.blue() >=1.0) {
                //          servoBeaconPinion.setPosition(1.0);
                //
                break;

            case DUMPCLIMBERS:
                if (dumperCounter >= dumperCounterThresh) {
                    dumperPosition -= .04;
                    servoClimberDumper.setPosition(dumperPosition);
                    dumperCounter = 0;
//             Target position reached; moving to next state
                    if (dumperPosition <= .25) {
                        nextMove = MoveState.ROTATEFROMBEACON;
                        currentMove = MoveState.MOVEDELAY;
                        telemetryMove = MoveState.DUMPTRUCK;
                        moveDelayTime = delay;
                        dumpedClimbers = true;
                        if (pushedButton == true) {
                            currentMove = MoveState.ROTATEFROMBEACON;
                        } else if (pushedButton == false) {
                            currentMove = MoveState.PUSHBUTTON;
                        }
                    }
                }
                dumperCounter++;
                break;

//             case CENTERBUCKET:
//                 lookingForRedFlag = false;
//                 lookingForBlueFlag = false;
//                 if ((redAlliance && sawRedFlag) || (!redAlliance && sawBlueFlag)) {
//                     if (redAlliance) {
//                         moveStraight(-10.0, fastSpeed);
//                     } else {
//                         moveStraight(10.0, fastSpeed);
//                     }
//                     currentMove = MoveState.STARTMOVE;
//                     nextMove = MoveState.DUMPTRUCK;
//                     telemetryMove = MoveState.CENTERBUCKET;
//                     moveDelayTime = delay;
//                 } else {
//                     currentMove = MoveState.DUMPTRUCK;
//                 }
//                 break;

//             case DUMPTRUCK:
//            If timer hits threshold, reset timer and move servo
//         if (dumperCounter >= dumperCounterThresh) {
//             dumperPosition -= .04;
//            servoClimberDumper.setPosition(dumperPosition);
//            dumperCounter = 0;
//             Target position reached; moving to next state
//            if (dumperPosition <= .25) {
//                nextMove = MoveState.ROTATEFROMBEACON;
//                currentMove = MoveState.MOVEDELAY;
//                telemetryMove = MoveState.DUMPTRUCK;
//                moveDelayTime = 1000;
//            }
//        }
//        dumperCounter++;
//        break;

            case ROTATEFROMBEACON:
                if (redAlliance) {
                    moveTurn(-45.0, turnSpeed);
                } else {
                    moveTurn(-135, turnSpeed);
                }
                servoClimberDumper.setPosition(1.0);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.MOVETORAMP;
                telemetryMove = MoveState.ROTATEFROMBEACON;
                moveDelayTime = delay;
                break;

            case MOVETORAMP:
                if (redAlliance) {
                    distance = -114.0;
                } else {
                    distance = -96.0;
                }
                if (!nearMountainFlag) {
                    distance -= 61.0;
                }
                moveStraight(distance, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTORAMP;
                telemetryMove = MoveState.MOVETORAMP;
                moveDelayTime = delay;
                break;

            case TURNTORAMP:
                if (nearMountainFlag) {
                    if (redAlliance) {
                        moveTurn(-90.0, turnSpeed);
                    } else {
                        moveTurn(90.0, turnSpeed);
                    }
                } else {
                    if (redAlliance) {
                        moveTurn(90.0, turnSpeed);
                    } else {
                        moveTurn(-90.0, turnSpeed);
                    }
                }
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.STOPATRAMP;
                telemetryMove = MoveState.TURNTORAMP;
                moveDelayTime = delay;
                break;

            case STOPATRAMP:
                if (nearMountainFlag) {
                    moveStraight(40.0, fastSpeed);
                } else {
                    moveStraight(200.0, fastSpeed);
                }
                servoUltraSense.setPosition(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.UPRAMP;
                telemetryMove = MoveState.STOPATRAMP;
                break;

            case UPRAMP:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                    moveStraight(distanceToWall - 5.0, slowSpeed);
                    //place code for extending omni wheels here
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DONE;
                    telemetryMove = MoveState.UPRAMP;
                }
                break;

            case DONE:
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                telemetryMove = MoveState.DONE;
                break;
        }
//        switch (currentMove) {
//
//            case PRETURN:
//                leftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//                rightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//                currentMove = turnMove;
//                break;
//
//            case STARTSTRAIGHTMOVE:
//                if (leftDrive.isBusy() && rightDrive.isBusy()) {
//                    currentMove = MoveState.STRAIGHTMOVING;
//                }
//                break;
//
//            case STRAIGHTMOVING:
//                if (lookingForFlag && ((beaconColorSense.red() >= 1) || (beaconColorSense.blue() >= 1))) {
//                    leftDrive.setPower(0.0);
//                    rightDrive.setPower(0.0);
//                    currentMove = MoveState.MOVEDELAY;
//                }
//                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
//                    currentMove = MoveState.MOVEDELAY;
//                }
//                break;
//
//            case ENCODERTURNMOVING:
//                if (lookingForFlag && ((beaconColorSense.red() >= 1) || (beaconColorSense.blue() >= 1))) {
//                    leftDrive.setPower(0.0);
//                    rightDrive.setPower(0.0);
//                    currentMove = MoveState.MOVEDELAY;
//                }
//                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
//                    currentMove = MoveState.MOVEDELAY;
//                }
//                moveDelayTime = 75;
//                break;
//
//
//            case SERVOPINIONNULLMOVE:
//                if (beaconColorSense.red() < 1.0 && beaconColorSense.blue() < 1.0) {
//                    moveBeaconPinion(0.0);
//                    currentMove = MoveState.MOVEDELAY;
//                } else if (beaconColorSense.red() >= 1.0 || beaconColorSense.blue() >= 1.0) {
//                    moveBeaconPinion(beaconPinionPosition);
//                    currentMove = MoveState.SERVOPINIONNULLMOVE;
//                }
//                break;
//
//            case SERVOPINIONREDMOVE:
//                if (beaconColorSense.red() >= 1.0) {
//                    moveBeaconPinion(0.0);
//                    currentMove = MoveState.MOVEDELAY;
//                } else if (beaconColorSense.red() < 1.0) {
//                    moveBeaconPinion(beaconPinionPosition);
//                    currentMove = MoveState.SERVOPINIONREDMOVE;
//                }
//
//            case MOVEDELAY:
//                now = new Date();
//                delayUntil = now.getTime() + moveDelayTime;
//                currentMove = MoveState.DELAY;
//                leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//                rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//                break;
//
//            case DELAY:
//                now = new Date();
//                if (now.getTime() >= delayUntil) {
//                    currentMove = nextMove;
//                }
//                break;
//
//            case FIRSTMOVE:
//                preTurnHeading = gyroSense.getHeading();
//                moveStraight(80.0, 0.4);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.PRETURN;
//                telemetryMove = MoveState.FIRSTMOVE;
//                moveDelayTime = 100;
//                break;
//
//            case TURNDIAGWITHENCODERS:
//                moveTurn(45, 0.5);
//                currentMove = MoveState.STARTENCODERTURN;
//                nextMove = MoveState.TURNDIAGWITHGYRO;
//                break;
//
//            case TURNDIAGWITHGYRO:
//                moveTurnWithGyro(45, true);
//                currentMove = MoveState.MOVEDELAY;
//                moveDelayTime = 100;
//                nextMove = MoveState.MOVEDIAG;
//                telemetryMove = MoveState.TURNDIAGWITHGYRO;
//                break;
//
//            case MOVEDIAG:
//                preTurnHeading = gyroSense.getHeading();
//                moveStraight(171.0, 0.4);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.FINDWALL;
//                telemetryMove = MoveState.MOVEDIAG;
//                moveDelayTime = 250;
//                break;
//
//            case FINDWALL:
//                distanceToWall = UltraSense.getUltrasonicLevel();
//                if ((distanceToWall > 30.0) && (distanceToWall <= 90.0))
//                {
//                    moveStraight((distanceToWall - 33.0) * 1.414, 0.4);
//                    currentMove = MoveState.STARTSTRAIGHTMOVE;
//                    nextMove = MoveState.PRETURN;
//                    telemetryMove = MoveState.FINDWALL;
//                    moveDelayTime = 250;
//                }
//                break;
//
//            case TURNALONGWALLWITHGYRO:
//                moveTurnWithGyro(135, true);
//                currentMove = MoveState.MOVEDELAY;
//                moveDelayTime = 100;
//                nextMove = MoveState.DRIVEALONGWALL;
//                telemetryMove = MoveState.TURNALONGWALLWITHGYRO;
//                break;
//
//            case DRIVEALONGWALL:
//                preTurnHeading = gyroSense.getHeading();
//                moveStraight(50.0, 0.4);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.FINDBEACON;
//                telemetryMove = MoveState.DRIVEALONGWALL;
//                moveDelayTime = 100;
//                break;
//
//            case FINDBEACON:
//                moveStraight(-30, 0.20);
//                lookingForFlag = true;
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.ALIGNDUMPER;
//                telemetryMove = MoveState.FINDBEACON;
//                moveDelayTime = 500;
//                break;
//
//            case ALIGNDUMPER:
//                ifRedOnBeacon = beaconColorSense.red();
//                ifBlueOnBeacon = beaconColorSense.blue();
//                moveBeaconPinion(0.5);
//                currentMove = MoveState.SERVOPINIONNULLMOVE;
//                nextMove = MoveState.DUMPCLIMBERS;
//                telemetryMove = MoveState.ALIGNDUMPER;
//                moveDelayTime = 3000;
//                break;
//
//            case DUMPCLIMBERS:
//                moveClimberDump(0.0);
//                moveClimberDump(1.0);
//                currentMove = MoveState.MOVEDELAY;
//                nextMove = MoveState.ALIGNPRESSER;
//                telemetryMove = MoveState.DUMPCLIMBERS;
//                moveDelayTime = 3000;
//                break;
//
//            case ALIGNPRESSER:
//                if (ifRedOnBeacon >= 1.0) {
//                    moveBeaconPinion(-0.5);
//                } else if (ifRedOnBeacon < 1.0) {
//                    moveBeaconPinion(0.5);
//                }
//                currentMove = MoveState.SERVOPINIONREDMOVE;
//                nextMove = MoveState.PRESSBUTTON;
//                telemetryMove = MoveState.ALIGNPRESSER;
//                moveDelayTime = 3000;
//                break;
//
//            case PRESSBUTTON:
//                moveBeaconPress(0.0);
//                moveBeaconPress(1.0);
//                currentMove = MoveState.MOVEDELAY;
//                nextMove = MoveState.PULLAHEADALONGWALL;
//                telemetryMove = MoveState.PRESSBUTTON;
//                moveDelayTime = 3000;
//                break;
//
//            case PULLAHEADALONGWALL:
//                moveStraight(30, 0.4);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.PRETURN;
//                telemetryMove = MoveState.PULLAHEADALONGWALL;
//                moveDelayTime = 200;
//                break;
//
//            case ROTATEFROMBEACONWITHGYRO:
//                moveTurnWithGyro(50, true);
//                currentMove = MoveState.MOVEDELAY;
//                moveDelayTime = 100;
//                lookingForFlag = false;
//                nextMove = MoveState.MOVETORAMP;
//                telemetryMove = MoveState.ROTATEFROMBEACONWITHGYRO;
//                break;
//
//            case MOVETORAMP:
//                preTurnHeading = gyroSense.getHeading();
//                moveStraight(-103.0, 0.4);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.PRETURN;
//                telemetryMove = MoveState.MOVETORAMP;
//                moveDelayTime = 100;
//                break;
//
//            case TURNTORAMPWITHGYRO:
//                moveTurnWithGyro(-101, true);
//                currentMove = MoveState.MOVEDELAY;
//                moveDelayTime = 100;
//                nextMove = MoveState.UPRAMP;
//                telemetryMove = MoveState.TURNTORAMPWITHGYRO;
//                break;
//
//            case UPRAMP:
//                moveStraight(0, 0.3);
//                currentMove = MoveState.STARTSTRAIGHTMOVE;
//                nextMove = MoveState.DONE;
//                telemetryMove = MoveState.UPRAMP;
//                break;
//
//            case DONE:
//                leftDrive.setPower(0.0);
//                rightDrive.setPower(0.0);
//                telemetryMove = MoveState.DONE;
//                break;
//        }

        telemetry.addData("Ultrasonic", ultraSense.getUltrasonicLevel());
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("ENCLeft", (float) leftDrive.getCurrentPosition());
        telemetry.addData("ENCRight", (float) rightDrive.getCurrentPosition());
        telemetry.addData("Gyro Heading", gyroSense.getHeading());
        telemetry.addData("delay pot", delayTime);
        telemetry.addData("elapsed time", matchTime.time());
    }

    @Override
    public void stop() {
    }
}

