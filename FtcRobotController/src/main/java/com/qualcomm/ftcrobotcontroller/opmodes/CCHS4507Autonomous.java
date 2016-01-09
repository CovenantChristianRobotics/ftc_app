package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;
/**
 * Created by cchsrobochargers on 12/17/15.
 */
public class CCHS4507Autonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, STARTTURN, TURNING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, CENTERBUCKET, DUMPTRUCK, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, STOPATRAMP, UPRAMP, DONE
    }

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    //servos
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    ColorSensor ColorSense;
    ColorSensor colorGroundSense;
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    boolean sawBlueFlag;
    boolean sawRedFlag;
    long delayUntil;
    double speed;
    boolean movingForward;
    double fastSpeed;
    double slowSpeed;
    double turnSpeed;
    long delay;
    Date now;
    GyroSensor gyroSense;
    int gyroError;
    int desiredHeading;
    UltrasonicSensor ultraSense;

    //Global State Variables
    int dumperCounter = 0;
    double dumperPosition = 0.9;

    // robot constants
//    double wheelDiameter = 6.75 / 2.0;  // wheel diameter in cm 2 to 1 gear ratio
//    double encoderCounts = 1120.0;      // encoder counts per revolution of the drive train motors
//    double wheelBase = 41.0;            // wheelbase of the primary drive wheels
    double countsPerMeter = 10439.0;    // Found this experimentally: Measured one meter, drove distance, read counts
    int dumperCounterThresh = 8;       // Doesn't let the dumper counter get above a certain number
    double countsPerDonut = 14161.0;    // Encoder counts per 360 degrees

    // Switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    // DigitalChannel delaySwitch;
    // DigitalChannel tileSwitch;

    // Analog Inputs
    AnalogInput delayPot;
    OpticalDistanceSensor liftCheck;
    int trackLifterUp = 0;
    boolean nearMountainFlag = false;
    boolean redAlliance = false;
   // long delayTimeFlag = 10;
   // double tileFlag = 1.0;


    public CCHS4507Autonomous() {
    }


    int centimetersToCounts(double centimeters) {
        return (int)(centimeters * (countsPerMeter / 100.0));
        //double wheelDiameter = 10.1;    // wheel diameter in cm
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
        //servos
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
        ColorSense = hardwareMap.colorSensor.get("color");
        colorGroundSense = hardwareMap.colorSensor.get("colorGround");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        // delaySwitch = hardwareMap.digitalChannel.get("dSw")
        // tileSwitch = hardwareMap.digitalChannel.get("tSw")
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        liftCheck = hardwareMap.opticalDistanceSensor.get("liftCheck");
        delayPot = hardwareMap.analogInput.get("delayPot");
        moveDelayTime = (long)(delayPot.getValue() * (15000 / 1024));
        nearMountainFlag = nearMountainSwitch.getState();
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

        if (liftCheck.getLightDetected() > 0.3) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }
        // tileFlag = tileSwitch.getState();
        //if (tileSwitch.getState()) {

        // } else {

        // }
        ColorSense.enableLed(true);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        nextMove = MoveState.FIRSTMOVE;
        currentMove = MoveState.MOVEDELAY;
        telemetryMove = MoveState.MOVEDELAY;
        sawRedFlag = false;
        sawBlueFlag = false;
        desiredHeading = 0;
        speed = 0;
        movingForward = true;
        fastSpeed = 0.95;
        slowSpeed = 0.75;
        turnSpeed = 0.75;
        delay = 50;
        trackLifter.setPower(0.1);
        trackLifter.setTargetPosition(30);
        servoClimberDumper.setPosition(0.9);
        servoBeaconPusher.setPosition(0.0);
        if (liftCheck.getLightDetected() > 0.3) {
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
                gyroError =  gyroSense.getHeading() - desiredHeading;
                if(gyroError > 180) {
                    gyroError = 360 - gyroError;
                }
                if (gyroError < -180) {
                    gyroError = 360 + gyroError;
                }
//                moveTurn(gyroError, speed);
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case MOVEDELAY:
                now = new Date();
                delayUntil = now.getTime() + moveDelayTime;
                currentMove = MoveState.DELAY;
                break;

            case DELAY:
                if (motorLeft.isBusy() || motorRight.isBusy()) {
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
                    moveTurn(45.0, turnSpeed); //origanal is -45
                } else {
                    moveTurn(135.0, turnSpeed);
                }
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.FINDBEACON;
                telemetryMove = MoveState.TURNALONGWALL;
                moveDelayTime = delay;
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
                moveDelayTime = delay;
                break;

            case CENTERBUCKET:
                lookingForRedFlag = false;
                lookingForBlueFlag = false;
                if ((redAlliance && sawRedFlag) || (!redAlliance && sawBlueFlag)) {
                    if (redAlliance) {
                        moveStraight(-10.0, fastSpeed);
                    } else {
                        moveStraight(10.0, fastSpeed);
                    }
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DUMPTRUCK;
                    telemetryMove = MoveState.CENTERBUCKET;
                    moveDelayTime = delay;
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
                moveDelayTime = delay;
                break;

            case MOVETORAMP:
                if (redAlliance) {
                    distance = -114.0;
                } else {
                    distance = -96.0;
                }
                if (!sawBlueFlag) {
                    distance += 10.0; // saw red first
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
                servoDist.setPosition(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.UPRAMP;
                telemetryMove = MoveState.STOPATRAMP;
                break;

            case UPRAMP:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                    moveStraight(distanceToWall - 5.0, slowSpeed);
                    trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    trackLifter.setPower(0.0);
                    trackLifter.setPowerFloat();
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DONE;
                    telemetryMove = MoveState.UPRAMP;
                }
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                telemetryMove = MoveState.DONE;
                break;
        }

        if (gamepad1.start) {
            currentMove = MoveState.FIRSTMOVE;
        }
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("Color", (float)ColorSense.red());
        telemetry.addData("desiredHeading", (float)desiredHeading);
        telemetry.addData("gyro", (float)gyroSense.getHeading());
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
        telemetry.addData("liftCheck", liftCheck.getLightDetected());
        telemetry.addData("delayPot", delayPot.getValue());
    }

    @Override
    public void stop() {
    }
}

