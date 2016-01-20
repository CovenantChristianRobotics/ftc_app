package com.qualcomm.ftcrobotcontroller.opmodes;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import java.util.Date;
/**
 * Created by cchsrobochargers on 12/17/15.
 */
public class CCHS4507Autonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, STARTTURN, TURNING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, CENTERBUCKET, DUMPTRUCK, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, STOPATRAMP, ALIGNRAMP, UPRAMP, DONE
    }

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    DcMotor armPivot;
    //servos
    //Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    Servo climberTriggerLeft;
    Servo climberTriggerRight;
    //Servo zipTieSweeper;
    ColorSensor ColorSense;
    ColorSensor colorGroundSense;
    TouchSensor touchSense;
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    boolean sawBlueFlag;
    boolean sawRedFlag;
    boolean fourthTileFlag;
    long delayUntil;
    double speed;
    boolean movingForward;
    double fastSpeed;
    double slowSpeed;
    double turnSpeed;
    long delayMillisec;
    long now;
    GyroSensor gyroSense;
    int gyroError;
    int desiredHeading;
    UltrasonicSensor ultraSense;

    //Global State Variables
    int dumperCounter = 0;
    double dumperPosition = 1.0;

    // robot constants
//    double wheelDiameter = 6.75 / 2.0;  // wheel diameter in cm 2 to 1 gear ratio
//    double encoderCounts = 1120.0;      // encoder counts per revolution of the drive train motors
//    double wheelBase = 41.0;            // wheelbase of the primary drive wheels
    double countsPerMeter = 5429.0; // 10439;    // Found this experimentally: Measured one meter, drove distance, read counts
    int dumperCounterThresh = 8;       // Doesn't let the dumper counter get above a certain number
    double countsPerDonut = 14161;    // Encoder counts per 360 degrees

    // Switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    // DigitalChannel delaySwitch;
    DigitalChannel fourthTileSwitch;

    // Analog Inputs
    AnalogInput delayPot;
    TouchSensor liftCheck;
    int trackLifterUp = 0;
    boolean nearMountainFlag = false;
    boolean redAlliance = false;
   // long delayTimeFlag = 10;
   // double tileFlag = 1.0;


    public CCHS4507Autonomous() {
    }


    int centimetersToCounts(double centimeters) {
        return (int)(centimeters * (countsPerMeter / 100.0));
        //double wheelDiameteliftCheckr = 10.1;    // wheel diameter in cm
        //double encoderCounts = 1120.0;  // encoder counts per revolution of the drive train motors
        // return (int) ((centimeters / (wheelDiameter * Math.PI)) * encoderCounts);
        // ^ Previous code to find the amount of counts for every wheel rotation
    }

    double countsToCentimeters(int counts) {
        return (double)counts * (100.0 / countsPerMeter);
        //return (((double) counts / encoderCounts) * (wheelDiameter * Math.PI));
        // ^ Previous code to find the amount of centimeters for the counts
    }

    int degreesToCounts(double degrees) {
        return (int)(degrees * (countsPerDonut / 360.0));
        //double wheelBase = 40.3225; // wheelbase of the primary drive wheels
        //double oneDegree = ((wheelBase * Math.PI) / 360);   // calculates the distance of one degree
        //return centimetersToCounts(oneDegree * degrees);
        // ^ Previous code to find the degrees per count using the diamemter of the wheels
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
        leftTarget = motorLeft.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorLeft.setTargetPosition(leftTarget);
        rightTarget = motorRight.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorRight.setTargetPosition(rightTarget);
        motorLeft.setPower(targetSpeed);
        motorRight.setPower(targetSpeed);
    }

    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees
     * @param targetSpeed
     */
    void moveTurn(double degrees, double targetSpeed) {
        int rightTarget;
        int leftTarget;

        // Figure out how far off we are at the end of the previous move so we can correct
        gyroError =  desiredHeading - gyroSense.getHeading();
        if(gyroError > 180) {
            gyroError = 360 - gyroError;
        }
        if (gyroError < -180) {
            gyroError = 360 + gyroError;
        }

        desiredHeading = desiredHeading + (int)degrees;
        if (desiredHeading >= 360) {
            desiredHeading = desiredHeading - 360;
        }
        if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
        }
        speed = targetSpeed;
        leftTarget = motorLeft.getCurrentPosition() - degreesToCounts(degrees + gyroError);
        motorLeft.setTargetPosition(leftTarget);
        rightTarget = motorRight.getCurrentPosition() + degreesToCounts(degrees + gyroError);
        motorRight.setTargetPosition(rightTarget);
        motorLeft.setPower(targetSpeed);
        motorRight.setPower(targetSpeed);
    }

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");
        armPivot = hardwareMap.dcMotor.get("armPivot");
        //servos
        //servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
        climberTriggerLeft = hardwareMap.servo.get("trigLeft");
        climberTriggerRight = hardwareMap.servo.get("trigRight");
        //zipTieSweeper = hardwareMap.servo.get("zipTieSweeper");
        ColorSense = hardwareMap.colorSensor.get("color");
        colorGroundSense = hardwareMap.colorSensor.get("colorGround");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        // delaySwitch = hardwareMap.digitalChannel.get("dSw")
        fourthTileSwitch = hardwareMap.digitalChannel.get("fourthTileSw");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        liftCheck =  hardwareMap.touchSensor.get("liftCheck");
        delayPot = hardwareMap.analogInput.get("delayPot");
        moveDelayTime = (long)(delayPot.getValue() * (15000 / 1024));
        nearMountainFlag = nearMountainSwitch.getState();
        fourthTileFlag = fourthTileSwitch.getState();
        if (redBlueSwitch.getState()) { //This is for when we're going to blue
            redAlliance = false;
            lookingForRedFlag = false;
            lookingForBlueFlag = true;
            servoDist.setPosition(0.75);
        } else { //This is for red
            redAlliance = true;
            lookingForRedFlag = true;
            lookingForBlueFlag = false;
            servoDist.setPosition(0.25);
        }

        if (liftCheck.isPressed()) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }
        // tileFlag = tileliftCheckSwitch.getState();
        //if (tileSwitch.getState()) {

        // } else {

        // }
        ColorSense.enableLed(true);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        armPivot.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        nextMove = MoveState.FIRSTMOVE;
        currentMove = MoveState.MOVEDELAY;
        telemetryMove = MoveState.MOVEDELAY;
        sawRedFlag = false;
        sawBlueFlag = false;
        desiredHeading = 0;
        speed = 0;
        movingForward = true;
        fastSpeed = 0.50;
        slowSpeed = 0.35;
        turnSpeed = 0.35;
        delayMillisec = 1000;
        //zipTieSweeper.setPosition(.75);
        trackLifter.setPower(0.1);
        trackLifter.setTargetPosition(30);
        servoClimberDumper.setPosition(1.0);
        climberTriggerLeft.setPosition(0.5);
        climberTriggerRight.setPosition(0.5);
        armPivot.setPower(0.0);
        //servoBeaconPusher.setPosition(0.0);
        if (liftCheck.isPressed()) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }
        gyroSense.calibrate();
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
                if (motorLeft.isBusy() && motorRight.isBusy()) {
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
                motorRight.setPower(Range.clip(speed + (gyroError * 0.2), -1.0, 1.0));
                motorLeft.setPower(Range.clip(speed - (gyroError * 0.2), -1.0, 1.0));
                if (ColorSense.blue() >= 1) {
                    sawBlueFlag = true;
                }
                if (ColorSense.red() >= 1) {
                    sawRedFlag = true;
                }
                if (lookingForRedFlag && (ColorSense.red() >= 1))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (lookingForBlueFlag && (ColorSense.blue() >= 1))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case STARTTURN:
                if (motorLeft.isBusy() && motorRight.isBusy()) {
                    currentMove = MoveState.TURNING;
                }
                break;

            case TURNING:
//                gyroError =  gyroSense.getHeading() - desiredHeading;
//                if(gyroError > 180) {
//                    gyroError = 360 - gyroError;
//                }
//                if (gyroError < -180) {
//                    gyroError = 360 + gyroError;
//                }
//                moveTurn(gyroError, speed);
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case MOVEDELAY:
//                now = new Date();
//                delayUntil = now.getTime() + moveDelayTime;
                now = System.currentTimeMillis();
                delayUntil = now + moveDelayTime;
                currentMove = MoveState.DELAY;
                Log.i("MOVEDELAY: time", Long.toString(System.currentTimeMillis()));
                Log.i("MOVEDELAY: now", Long.toString(now));
                Log.i("MOVEDELAY: moveDelayTime", Long.toString(moveDelayTime));
                Log.i("MOVEDELAY: delayUntil", Long.toString(delayUntil));

                break;

            case DELAY:
//                if (motorLeft.isBusy() || motorRight.isBusy()) {
//                    // If we aren't quite done moving, restart the delayMillisec
//                    currentMove = MoveState.MOVEDELAY;
//                } else {
//                    now = new Date();
//                    if (now.getTime() >= delayUntil) {
                    if (System.currentTimeMillis() >= delayUntil) {
                        currentMove = nextMove;
                    }
//                }
                break;

            case FIRSTMOVE:
                moveStraight(60.0 + (fourthTileFlag ? 45.0 : 0.0), fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = delayMillisec;
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
                moveDelayTime = delayMillisec;
                break;

            case MOVEDIAG:
                moveStraight(209.0 - (fourthTileFlag ? 60.0 : 0.0), fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDWALL;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = delayMillisec;
                break;

            case FINDWALL:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 20.0) && (distanceToWall <= 80.0)) {
                    if (redAlliance) {
                        moveStraight((distanceToWall - 26.0) * 1.414, slowSpeed);
                    } else {
                        moveStraight((distanceToWall - 30.0) * 1.414, slowSpeed);
                    }
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNALONGWALL;
                    telemetryMove = MoveState.FINDWALL;
                    moveDelayTime = delayMillisec;
                }
                break;

            case TURNALONGWALL:
                if (redAlliance) {
                    moveTurn(45.0, turnSpeed); //origanal is -45
                } else {
                    moveTurn(135.0, turnSpeed);
                }
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.FINDBEACON;
                telemetryMove = MoveState.TURNALONGWALL;
                moveDelayTime = delayMillisec;
                break;

            case FINDBEACON:
                if (redAlliance) {
                    moveStraight(-90.0, fastSpeed);
                } else {
                    moveStraight(90.0, fastSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CENTERBUCKET;
                telemetryMove = MoveState.FINDBEACON;
                moveDelayTime = delayMillisec;
                break;

            case CENTERBUCKET:
                lookingForRedFlag = false;
                lookingForBlueFlag = false;
                if ((redAlliance && !sawBlueFlag) || (!redAlliance && !sawRedFlag)) {
                    if (redAlliance) {
                        moveStraight(-10.0, fastSpeed);
                    } else {
                        moveStraight(10.0, fastSpeed);
                    }
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DUMPTRUCK;
                    telemetryMove = MoveState.CENTERBUCKET;
                    moveDelayTime = delayMillisec;
                } else {
                    currentMove = MoveState.DUMPTRUCK;
                }
                break;

            case DUMPTRUCK:
                // If timer hits threshold, reset timer and move servo
                if (dumperCounter >= dumperCounterThresh) {
                    dumperPosition -= .04;
                    servoClimberDumper.setPosition(dumperPosition);
                    dumperCounter = 0;
                    // Target position reached; moving to next state
                    if (dumperPosition <= .25) {
                        nextMove = MoveState.ROTATEFROMBEACON;
                        currentMove = MoveState.MOVEDELAY;
                        telemetryMove = MoveState.DUMPTRUCK;
                        moveDelayTime = 1000;
                    }
                }
                dumperCounter++;
                break;

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
                moveDelayTime = delayMillisec;
                break;

            case MOVETORAMP:
                if (redAlliance) {      // red alliance
                    distance = -108.0;
                } else {                // blue alliance
                    distance = -89.0;
                }
                if (!nearMountainFlag) {
                    distance -= 61.0;
                }
                moveStraight(distance, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTORAMP;
                telemetryMove = MoveState.MOVETORAMP;
                moveDelayTime = delayMillisec;
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
                moveDelayTime = delayMillisec;
                break;

            case STOPATRAMP:
                if (nearMountainFlag) {
                    moveStraight(40.0, fastSpeed);
                } else {
                    moveStraight(200.0, fastSpeed);
                }
                servoDist.setPosition(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.ALIGNRAMP;
                telemetryMove = MoveState.STOPATRAMP;
                break;

            case ALIGNRAMP:
                moveTurn(0.0, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.UPRAMP;
                telemetryMove = MoveState.ALIGNRAMP;
                break;

            case UPRAMP:
//                distanceToWall = ultraSense.getUltrasonicLevel();
//                if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                    moveStraight(100.0, slowSpeed);
                    trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    trackLifter.setPower(0.0);
                    trackLifter.setPowerFloat();
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DONE;
                    telemetryMove = MoveState.UPRAMP;
//                }
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                trackLifter.setPower(0.1);
                trackLifter.setTargetPosition(30);
                telemetryMove = MoveState.DONE;
                break;
        }

        if (gamepad1.start) {
            currentMove = MoveState.FIRSTMOVE;
        }
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("desiredHeading", (float)desiredHeading);
        telemetry.addData("gyro", (float)gyroSense.getHeading());
        telemetry.addData("time", (float)System.currentTimeMillis());
        telemetry.addData("delayUntil", (float)delayUntil);
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
        telemetry.addData("liftCheck", liftCheck.isPressed());
        telemetry.addData("delayPot", delayPot.getValue());

        Log.i("Current Move", currentMove.toString());
        Log.i("desiredHeading", Integer.toString(desiredHeading));
        Log.i("gyro", Integer.toString(gyroSense.getHeading()));
        Log.i("time", Long.toString(System.currentTimeMillis()));
        Log.i("delayUntil", Long.toString(delayUntil));
    }

    @Override
    public void stop() {
    }
}

