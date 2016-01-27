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
 * Created by Robotics on 1/7/2016.
 */
public class testgyroturn extends OpMode {



/**
 * Created by Robotics on 10/7/2015.
 */

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
    int nearMtn;
    int redBlue;
    int delay;
    long delayTime;
    //    int tile;
    ElapsedTime matchTime;

    //delayMillisec settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    public testgyroturn () {
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
                    leftDrive.setPower(Range.clip((-(difference / 100) * 3) * redBlue, -1.0, 1.0));
                    rightDrive.setPower(Range.clip(((difference / 100) * 3) * redBlue, -1.0, 1.0));
                } else if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower(Range.clip(((difference / 100) * 3) * redBlue, -1.0, 1.0));
                    rightDrive.setPower(Range.clip((-(difference / 100) * 3) * redBlue, -1.0, 1.0));
                }
            } else if (reverseByColor == false){
                if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower(Range.clip((-(difference / 100) * 3), -1.0, 1.0));
                    rightDrive.setPower(Range.clip(((difference / 100) * 3), -1.0, 1.0));
                } else if (currentHeading + centerDist > turnTo + centerDist) {
                    leftDrive.setPower(Range.clip(((difference / 100) * 3), -1.0, 1.0));
                    rightDrive.setPower(Range.clip((-(difference / 100) * 3), -1.0, 1.0));
                }

            }
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
        currentMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        lookingForFlag = false;
        // switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
        delayPotSwitch = hardwareMap.digitalChannel.get("dSw");
        matchTime = new ElapsedTime();
        //servo positions
        moveBeaconPinion(0.0);
        moveClimberDump(1.0);
        moveBeaconPress(1.0);
        servoUltraSense.setPosition(0.25);
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
                nextMove = MoveState.TURNDIAGWITHGYRO;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = 100;
                break;

            case TURNDIAGWITHGYRO:
                moveTurnWithGyro(15, false);
                currentMove = MoveState.MOVEDELAY;
                moveDelayTime = 100;
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.TURNDIAGWITHGYRO;
                break;

            case DONE:
                moveStraight(0.0, 0);
                break;

        }

        telemetry.addData("Ultrasonic", UltraSense.getUltrasonicLevel());
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("ENCLeft", (float) leftDrive.getCurrentPosition());
        telemetry.addData("ENCRight", (float) rightDrive.getCurrentPosition());
        telemetry.addData("Gyro Heading", gyroSense.getHeading());
        telemetry.addData("delayMillisec pot", delayTime);
        telemetry.addData("elapsed time", matchTime.time());
    }

    @Override
    public void stop() {
    }
}

