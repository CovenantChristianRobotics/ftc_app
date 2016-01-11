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
    enum MoveState {
        DELAY, STARTSTRAIGHTMOVE, STRAIGHTMOVING, STARTENCODERTURN, ENCODERTURNMOVING, GYROTURN,
        PRETURN, SERVOPINIONREDMOVE, SERVOPINIONNULLMOVE, SERVOPINIONBLUEMOVE, SERVOPUSHERMOVE,
        SERVODUMPERMOVE, MOVEDELAY, FIRSTMOVE, TURNDIAGWITHGYRO, TURNDIAGWITHENCODERS, MOVEDIAG,
        FINDWALL, TURNALONGWALLWITHGYRO, TURNALONGWALLWITHENCODERS, DRIVEALONGWALL, FINDBEACON,
        ALIGNDUMPER, DUMPCLIMBERS, ALIGNPRESSER, PRESSBUTTON, PULLAHEADALONGWALL,
        ROTATEFROMBEACONWITHGYRO, ROTATEFROMBEACONWITHENCODERS, MOVETORAMP, TURNTORAMPWITHGYRO,
        TURNTORAMPWITHENCODERS, UPRAMP, DONE
    }

    // maximum and minimum values to use when clipping the ranges
    final static double bpusher_MIN_RANGE  = 0.20;
    final static double bpusher_MAX_RANGE  = 0.80;
    final static double cdumper_MIN_RANGE  = 0.00;
    final static double cdumber_MAX_RANGE  = 1.00;

    // target values for servos
    double beaconPusherPosition;
    double climberDumperPosition;
    double beaconPinionPosition;

    //dc motor controllers
    DcMotorController driveTrainController;
    DcMotorController hangingController;
    //dc motors
    DcMotor leftDrive;
    DcMotor rightDrive;
//    DcMotor chinUp;
    //servo controllers
    ServoController beaconController;
//    ServoController alignmentController;
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
    UltrasonicSensor UltraSense;
    TouchSensor beaconPinionStop;
//    TouchSensor leftWheelStop;
//    TouchSensor rightWheelStop;
    //Statemachine options
    MoveState currentMove;
    MoveState nextMove;
    MoveState turnMove;
    MoveState telemetryMove;
    double ifRedOnBeacon;
    double ifBlueOnBeacon;
    boolean lookingForFlag;
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
//    int tile;
    ElapsedTime matchTime;

    //delay settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    public CCHS5256Autonomous () {
    }

    int centimetersToCounts(double centimeters) {
        return (int)(36.09 * centimeters);
    }

    double countsToCentimeters(int counts) {
        return (int)((double)counts / 36.09);
    }

    int degreesToCounts(double degrees) {
        return (int)((6077.0 / 360.0) * degrees);
    }

    void moveStraight(double distanceCM, double speed) {
        int rightTarget;
        int leftTarget;

        leftTarget = leftDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees
     * @param speed
     */
    void moveTurn(double degrees, double speed) {
        int rightTarget;
        int leftTarget;

        leftTarget = leftDrive.getCurrentPosition() - degreesToCounts(degrees) ;
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + degreesToCounts(degrees);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees
     */
    void moveTurnWithGyro(int degrees, boolean reverseByColor) {
        int turnTo;
        int centerDist;
        int currentHeading;
        int difference;

        turnTo = preTurnHeading + degrees;
        if (turnTo > 359) {
            turnTo = turnTo - 360;
        } else if (turnTo < 0) {
            turnTo = turnTo + 360;
        }

        centerDist = 180 - turnTo;
        currentHeading = gyroSense.getHeading();

        while (currentHeading != turnTo) {
            difference = (turnTo + centerDist) - (currentHeading + centerDist);
            currentHeading = gyroSense.getHeading();

            if (reverseByColor == true) {
                if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower((-(difference / 100) * 3) * redBlue);
                    rightDrive.setPower(((difference / 100) * 3) * redBlue);
                } else if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower(((difference / 100) * 3) * redBlue);
                    rightDrive.setPower((-(difference / 100) * 3) * redBlue);
                }
            } else if (reverseByColor == false){
                if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower(-(difference / 100) * 3);
                    rightDrive.setPower((difference / 100) * 3);
                } else if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower((difference / 100) * 3);
                    rightDrive.setPower(-(difference / 100) * 3);
                }

            }
        }

        if (reverseByColor == true) {


        } else if (reverseByColor == false){

            }
        }




    void motorOn(double speed, boolean onoroff) {
        if(onoroff = true){
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
            leftDrive.setTargetPosition(1000000000);
            rightDrive.setTargetPosition(1000000000);
        }
        if (onoroff = false) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        }
    }

    /**
     * speed is between -1 and +1
     *
     * @param beaconPinionSpeed
     */
    void moveBeaconPinion(double beaconPinionSpeed) {
        double FbeaconPinionSpeed = ((beaconPinionSpeed / 2) + 0.5);
        servoBeaconPinion.setPosition(FbeaconPinionSpeed);
    }

    /**
     * since beaconPresserPosition is clipped, values can be entered between 0.0 and 0.1
     *
     * @param beaconPresserPosition
     */
    void moveBeaconPress(double beaconPresserPosition) {
        beaconPresserPosition = Range.clip(beaconPresserPosition, bpusher_MIN_RANGE, bpusher_MAX_RANGE);
        servoBeaconPusher.setPosition(beaconPresserPosition);
        beaconPusherPosition = beaconPresserPosition;
    }

    /**
     * since climberDumperPosition is clipped, values can be entered between 0.0 and 0.1
     *
     * @param climberDumperPosition
     */
    void moveClimberDump(double climberDumperPosition) {
        climberDumperPosition = Range.clip(climberDumperPosition, cdumper_MIN_RANGE, cdumber_MAX_RANGE);
        servoClimberDumper.setPosition(climberDumperPosition);
        climberDumperPosition = climberDumperPosition;
    }

    /**
     * speed is between -1 and +1
     *
     * @param lOmnipinionSpeed
     */
    void moveLeftOmnipinion(double lOmnipinionSpeed) {
        double FomniPinionSpeed = ((lOmnipinionSpeed / 2) + 0.5);
        servoBeaconPinion.setPosition(FomniPinionSpeed);
    }

    /**
     * speed is between -1 and +1
     *
     * @param rOmnipinionSpeed
     */
    void moveRightOmnipinion(double rOmnipinionSpeed) {
        double FomniPinionSpeed = ((rOmnipinionSpeed / 2) + 0.5);
        servoBeaconPinion.setPosition(FomniPinionSpeed);
    }

    @Override
    public void init() {
        //dc Motor Controllers
        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
        hangingController = hardwareMap.dcMotorController.get("hangCtlr");
        //dc Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
//        chinUp = hardwareMap.dcMotor.get("chinUp");
        //servo controllers
        beaconController = hardwareMap.servoController.get("beaconCtlr");
//        alignmentController = hardwareMap.servoController.get("alignCtlr");
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
        UltraSense = hardwareMap.ultrasonicSensor.get("UltraSense");
        beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
//        leftWheelStop = hardwareMap.touchSensor.get("lWStop");
//        rightWheelStop = hardwareMap.touchSensor.get("rWStop");
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
        delayTime = (long)(delayPotentiometer.getValue() * (10000/1024));
        matchTime = new ElapsedTime();
        //servo positions
        moveBeaconPinion(0.0);
        moveClimberDump(1.0);
        moveBeaconPress(1.0);
        servoUltraSense.setPosition(0.25);
        moveDelayTime = delayTime;
        if (redBlueBeaconSwitch.getState()) { //red alliance
            redBlue = 1;
            servoUltraSense.setPosition(.75);
        } else { // blue alliance
            redBlue = -1;
            servoUltraSense.setPosition(.25);
        }
        // align color sensor
         while (!beaconPinionStop.isPressed()) {
             moveBeaconPinion(0.5);
         }
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
        double distanceToWall;
        if (gyroSense.isCalibrating()) {
            return;
        }
        
//         switch (currentMove) {

//             case STARTMOVE:
//                 if (motorLeft.isBusy() && motorRight.isBusy()) {
//                     currentMove = MoveState.MOVING;
//                 }
//                 break;

//             case MOVING:
//                 gyroError = desiredHeading - gyroSense.getHeading();
//                 if (gyroError > 180) {
//                     gyroError = 360 - gyroError;
//                 }
//                 if (gyroError < -180) {
//                     gyroError = 360 + gyroError;
//                 }
//                 if (!movingForward) {
//                     gyroError = -gyroError;
//                 }
//                 motorRight.setPower(Range.clip(speed + (gyroError * 0.2), -1.0, 1.0));
//                 motorLeft.setPower(Range.clip(speed - (gyroError * 0.2), -1.0, 1.0));
//                 if (lookingForFlag && (ColorSense.red() >= 1))  {
//                     motorRight.setPower(0.0);
//                     motorLeft.setPower(0.0);
//                     currentMove = MoveState.MOVEDELAY;
//                 }
//                 if (lookingForFlag && (ColorSense.blue() >= 1))  {
//                     motorRight.setPower(0.0);
//                     motorLeft.setPower(0.0);
//                     currentMove = MoveState.MOVEDELAY;
//                 }
//                 if (!motorLeft.isBusy() && !motorRight.isBusy()) {
//                     currentMove = MoveState.MOVEDELAY;
//                 }
//                 break;

//             case STARTTURN:
//                 if (motorLeft.isBusy() && motorRight.isBusy()) {
//                     currentMove = MoveState.TURNING;
//                 }
//                 break;

//             case TURNING:
//                 gyroError =  gyroSense.getHeading() - desiredHeading;
//                 if(gyroError > 180) {
//                     gyroError = 360 - gyroError;
//                 }
//                 if (gyroError < -180) {
//                     gyroError = 360 + gyroError;
//                 }
// //                moveTurn(gyroError, speed);
//                 if (!motorLeft.isBusy() && !motorRight.isBusy()) {
//                     currentMove = MoveState.MOVEDELAY;
//                 }
//                 break;

//             case MOVEDELAY:
//                 now = new Date();
//                 delayUntil = now.getTime() + moveDelayTime;
//                 currentMove = MoveState.DELAY;
//                 break;

//             case DELAY:
//                 if (motorLeft.isBusy() || motorRight.isBusy()) {
//                     // If we aren't quite done moving, restart the delay
//                     currentMove = MoveState.MOVEDELAY;
//                 } else {
//                     now = new Date();
//                     if (now.getTime() >= delayUntil) {
//                         currentMove = nextMove;
//                     }
//                 }
//                 break;

//             case FIRSTMOVE:
//                 moveStraight(60.0, fastSpeed);
//                 currentMove = MoveState.STARTMOVE;
//                 nextMove = MoveState.TURNDIAG;
//                 telemetryMove = MoveState.FIRSTMOVE;
//                 moveDelayTime = delay;
//                 break;

//             case TURNDIAG:
//                 if (redAlliance) {
//                     moveTurn(-45.0, turnSpeed);
//                 } else {
//                     moveTurn(45.0, turnSpeed);
//                 }
//                 currentMove = MoveState.STARTTURN;
//                 nextMove = MoveState.MOVEDIAG;
//                 telemetryMove = MoveState.TURNDIAG;
//                 moveDelayTime = delay;
//                 break;

//             case MOVEDIAG:
//                 moveStraight(209.0, fastSpeed);
//                 currentMove = MoveState.STARTMOVE;
//                 nextMove = MoveState.FINDWALL;
//                 telemetryMove = MoveState.MOVEDIAG;
//                 moveDelayTime = delay;
//                 break;

//             case FINDWALL:
//                 distanceToWall = ultraSense.getUltrasonicLevel();
//                 if ((distanceToWall > 30.0) && (distanceToWall <= 80.0)) {
//                     if (redAlliance) {
//                         moveStraight((distanceToWall - 18.0) * 1.414, slowSpeed);
//                     } else {
//                         moveStraight((distanceToWall - 34.0) * 1.414, slowSpeed);
//                     }
//                     currentMove = MoveState.STARTMOVE;
//                     nextMove = MoveState.TURNALONGWALL;
//                     telemetryMove = MoveState.FINDWALL;
//                     moveDelayTime = delay;
//                 }
//                 break;

//             case TURNALONGWALL:
//                 if (redAlliance) {
//                     moveTurn(45.0, turnSpeed); //original is -45
//                 } else {
//                     moveTurn(135.0, turnSpeed);
//                 }
//                 currentMove = MoveState.STARTTURN;
//                 nextMove = MoveState.FINDBEACON;
//                 telemetryMove = MoveState.TURNALONGWALL;
//                 moveDelayTime = delay;
//                 break;

//             case FINDBEACON:
//                 if (redAlliance) {
//                     moveStraight(-90.0, fastSpeed);
//                 } else {
//                     moveStraight(90.0, fastSpeed);
//                 }
//                 currentMove = MoveState.STARTMOVE;
//                 nextMove = MoveState.CENTERBUCKET;
//                 telemetryMove = MoveState.FINDBEACON;
//                 moveDelayTime = delay;
//                 break;

//             case CENTERBUCKET:
//                 lookingForFlag = false;
//                     currentMove = MoveState.STARTMOVE;
//                     nextMove = MoveState.DUMPTRUCK;
//                     telemetryMove = MoveState.CENTERBUCKET;
//                     moveDelayTime = delay;
//                 } else {
//                     currentMove = MoveState.DUMPTRUCK;
//                 }
//                 break;

//             case DUMPTRUCK:
//                 // If timer hits threshold, reset timer and move servo
//                 if (dumperCounter >= dumperCounterThresh) {
//                     dumperPosition -= .04;
//                     servoClimberDumper.setPosition(dumperPosition);
//                     dumperCounter = 0;
//                     // Target position reached; moving to next state
//                     if (dumperPosition <= .25) {
//                         nextMove = MoveState.ROTATEFROMBEACON;
//                         currentMove = MoveState.MOVEDELAY;
//                         telemetryMove = MoveState.DUMPTRUCK;
//                         moveDelayTime = 1000;
//                     }
//                 }
//                 dumperCounter++;
//                 break;

//             case ROTATEFROMBEACON:
//                 if (redAlliance) {
//                     moveTurn(-45.0, turnSpeed);
//                 } else {
//                     moveTurn(-135, turnSpeed);
//                 }
//                 servoClimberDumper.setPosition(1.0);
//                 currentMove = MoveState.STARTTURN;
//                 nextMove = MoveState.MOVETORAMP;
//                 telemetryMove = MoveState.ROTATEFROMBEACON;
//                 moveDelayTime = delay;
//                 break;

//             case MOVETORAMP:
//                 if (redAlliance) {
//                     distance = -114.0;
//                 } else {
//                     distance = -96.0;
//                 }
//                 if (!nearMountainFlag) {
//                     distance -= 61.0;
//                 }
//                 moveStraight(distance, fastSpeed);
//                 currentMove = MoveState.STARTMOVE;
//                 nextMove = MoveState.TURNTORAMP;
//                 telemetryMove = MoveState.MOVETORAMP;
//                 moveDelayTime = delay;
//                 break;

//             case TURNTORAMP:
//                 if (nearMountainFlag) {
//                     if (redAlliance) {
//                         moveTurn(-90.0, turnSpeed);
//                     } else {
//                         moveTurn(90.0, turnSpeed);
//                     }
//                 } else {
//                     if (redAlliance) {
//                         moveTurn(90.0, turnSpeed);
//                     } else {
//                         moveTurn(-90.0, turnSpeed);
//                     }
//                 }
//                 currentMove = MoveState.STARTTURN;
//                 nextMove = MoveState.STOPATRAMP;
//                 telemetryMove = MoveState.TURNTORAMP;
//                 moveDelayTime = delay;
//                 break;

//             case STOPATRAMP:
//                 if (nearMountainFlag) {
//                     moveStraight(40.0, fastSpeed);
//                 } else {
//                     moveStraight(200.0, fastSpeed);
//                 }
//                 servoDist.setPosition(0.5);
//                 currentMove = MoveState.STARTMOVE;
//                 nextMove = MoveState.UPRAMP;
//                 telemetryMove = MoveState.STOPATRAMP;
//                 break;

//             case UPRAMP:
//                 distanceToWall = ultraSense.getUltrasonicLevel();
//                 if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
//                     moveStraight(distanceToWall - 5.0, slowSpeed);
//                     //place code for extending omni wheels here
//                     currentMove = MoveState.STARTMOVE;
//                     nextMove = MoveState.DONE;
//                     telemetryMove = MoveState.UPRAMP;
//                 }
//                 break;

//             case DONE:
//                 motorLeft.setPower(0.0);
//                 motorRight.setPower(0.0);
//                 telemetryMove = MoveState.DONE;
//                 break;
//         }
        switch (currentMove) {

            case PRETURN:
                leftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                rightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                currentMove = turnMove;
                break;

            case STARTSTRAIGHTMOVE:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.STRAIGHTMOVING;
                }
                break;

            case STRAIGHTMOVING:
                if (lookingForFlag && ((beaconColorSense.red() >= 1) || (beaconColorSense.blue() >= 1))) {
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case ENCODERTURNMOVING:
                if (lookingForFlag && ((beaconColorSense.red() >= 1) || (beaconColorSense.blue() >= 1))) {
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                moveDelayTime = 75;
                break;


            case SERVOPINIONNULLMOVE:
                if (beaconColorSense.red() < 1.0 && beaconColorSense.blue() < 1.0) {
                    moveBeaconPinion(0.0);
                    currentMove = MoveState.MOVEDELAY;
                } else if (beaconColorSense.red() >= 1.0 || beaconColorSense.blue() >= 1.0) {
                    moveBeaconPinion(beaconPinionPosition);
                    currentMove = MoveState.SERVOPINIONNULLMOVE;
                }
                break;

            case SERVOPINIONREDMOVE:
                if (beaconColorSense.red() >= 1.0) {
                    moveBeaconPinion(0.0);
                    currentMove = MoveState.MOVEDELAY;
                } else if (beaconColorSense.red() < 1.0) {
                    moveBeaconPinion(beaconPinionPosition);
                    currentMove = MoveState.SERVOPINIONREDMOVE;
                }

            case MOVEDELAY:
                now = new Date();
                delayUntil = now.getTime() + moveDelayTime;
                currentMove = MoveState.DELAY;
                leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                break;

            case DELAY:
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentMove = nextMove;
                }
                break;

            case FIRSTMOVE:
                preTurnHeading = gyroSense.getHeading();
                moveStraight(80.0, 0.4);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.PRETURN;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = 100;
                break;

            case TURNDIAGWITHENCODERS:
                moveTurn(45, 0.5);
                currentMove = MoveState.STARTENCODERTURN;
                nextMove = MoveState.TURNDIAGWITHGYRO;
                break;

            case TURNDIAGWITHGYRO:
                moveTurnWithGyro(45, true);
                currentMove = MoveState.MOVEDELAY;
                moveDelayTime = 100;
                nextMove = MoveState.MOVEDIAG;
                telemetryMove = MoveState.TURNDIAGWITHGYRO;
                break;

            case MOVEDIAG:
                preTurnHeading = gyroSense.getHeading();
                moveStraight(171.0, 0.4);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.FINDWALL;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = 250;
                break;

            case FINDWALL:
                distanceToWall = UltraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 90.0))
                {
                    moveStraight((distanceToWall - 33.0) * 1.414, 0.4);
                    currentMove = MoveState.STARTSTRAIGHTMOVE;
                    nextMove = MoveState.PRETURN;
                    telemetryMove = MoveState.FINDWALL;
                    moveDelayTime = 250;
                }
                break;

            case TURNALONGWALLWITHGYRO:
                moveTurnWithGyro(135, true);
                currentMove = MoveState.MOVEDELAY;
                moveDelayTime = 100;
                nextMove = MoveState.DRIVEALONGWALL;
                telemetryMove = MoveState.TURNALONGWALLWITHGYRO;
                break;

            case DRIVEALONGWALL:
                preTurnHeading = gyroSense.getHeading();
                moveStraight(50.0, 0.4);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.FINDBEACON;
                telemetryMove = MoveState.DRIVEALONGWALL;
                moveDelayTime = 100;
                break;

            case FINDBEACON:
                moveStraight(-30, 0.20);
                lookingForFlag = true;
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.ALIGNDUMPER;
                telemetryMove = MoveState.FINDBEACON;
                moveDelayTime = 500;
                break;

            case ALIGNDUMPER:
                ifRedOnBeacon = beaconColorSense.red();
                ifBlueOnBeacon = beaconColorSense.blue();
                moveBeaconPinion(0.5);
                currentMove = MoveState.SERVOPINIONNULLMOVE;
                nextMove = MoveState.DUMPCLIMBERS;
                telemetryMove = MoveState.ALIGNDUMPER;
                moveDelayTime = 3000;
                break;

            case DUMPCLIMBERS:
                moveClimberDump(0.0);
                moveClimberDump(1.0);
                currentMove = MoveState.MOVEDELAY;
                nextMove = MoveState.ALIGNPRESSER;
                telemetryMove = MoveState.DUMPCLIMBERS;
                moveDelayTime = 3000;
                break;

            case ALIGNPRESSER:
                if (ifRedOnBeacon >= 1.0) {
                    moveBeaconPinion(-0.5);
                } else if (ifRedOnBeacon < 1.0) {
                    moveBeaconPinion(0.5);
                }
                currentMove = MoveState.SERVOPINIONREDMOVE;
                nextMove = MoveState.PRESSBUTTON;
                telemetryMove = MoveState.ALIGNPRESSER;
                moveDelayTime = 3000;
                break;

            case PRESSBUTTON:
                moveBeaconPress(0.0);
                moveBeaconPress(1.0);
                currentMove = MoveState.MOVEDELAY;
                nextMove = MoveState.PULLAHEADALONGWALL;
                telemetryMove = MoveState.PRESSBUTTON;
                moveDelayTime = 3000;
                break;

            case PULLAHEADALONGWALL:
                moveStraight(30, 0.4);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.PRETURN;
                telemetryMove = MoveState.PULLAHEADALONGWALL;
                moveDelayTime = 200;
                break;

            case ROTATEFROMBEACONWITHGYRO:
                moveTurnWithGyro(50, true);
                currentMove = MoveState.MOVEDELAY;
                moveDelayTime = 100;
                lookingForFlag = false;
                nextMove = MoveState.MOVETORAMP;
                telemetryMove = MoveState.ROTATEFROMBEACONWITHGYRO;
                break;

            case MOVETORAMP:
                preTurnHeading = gyroSense.getHeading();
                moveStraight(-103.0, 0.4);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.PRETURN;
                telemetryMove = MoveState.MOVETORAMP;
                moveDelayTime = 100;
                break;

            case TURNTORAMPWITHGYRO:
                moveTurnWithGyro(-101, true);
                currentMove = MoveState.MOVEDELAY;
                moveDelayTime = 100;

                nextMove = MoveState.UPRAMP;
                telemetryMove = MoveState.TURNTORAMPWITHGYRO;

                break;

            case UPRAMP:
                moveStraight(0, 0.3);
                currentMove = MoveState.STARTSTRAIGHTMOVE;
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.UPRAMP;
                break;

            case DONE:
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                telemetryMove = MoveState.DONE;
                break;
        }

        telemetry.addData("Ultrasonic", UltraSense.getUltrasonicLevel());
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

