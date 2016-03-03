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
        FINDBEACON, MEASUREAMBIENT, MOVETOBUTTON, PRESSBUTTON,CENTERBUCKET, DUMPTRUCK, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, STOPATRAMP, ALIGNRAMP,
        DOWNTRACK, UPRAMP, STRAIGHTTORAMP, STRAIGHTTORAMPTURN, DONE
    }

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    //servos
    //Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoDist;
    Servo climberTriggerLeft;
    Servo climberTriggerRight;
    //Servo climberDoor;
    Servo cowCatcher;
    Servo armLock;
    Servo trackLock;
    //Servo zipTieSweeper;
    ColorSensor colorSense;
    //ColorSensor colorGroundSense;
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    int ambientRed;
    int ambientBlue;
    int lightAverageCounter;
    boolean sawBlueFlag;
    boolean sawRedFlag;
    boolean fourthTileFlag;
    boolean toMountainFlag;
    long delayUntil;
    int leftTargetPosition;
    int rightTargetPosition;
    double speed;
    boolean movingForward;
    boolean takeADump;
    boolean stowLifter;
    boolean noSquiggle;
    boolean cowCatcherUp = false;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    double turnSpeed;
    double xHeading;
    double yHeading;
    long gyroReadLast;
    long delayMillisec;
    long now;
    double distanceToWall;
    GyroSensor gyroSense;
    int gyroError;
    int desiredHeading;
    UltrasonicSensor ultraSense;

    // robot constants
    double countsPerMeter = 5361.0; // 10439;    // Found this experimentally: Measured one meter, drove distance, read counts
    int moveDoneDelta;              // How close we need to call it done
    double countsPerDonut = 7661.0; // 14161;    // Encoder counts per 360 degrees
    double trackLifterCountsPerDegree = -1170.0 * 2.0 / 90.0;

    // Switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    DigitalChannel toMountainSwitch;
    DigitalChannel fourthTileSwitch;

    // Analog Inputs
    AnalogInput delayPot;
    TouchSensor liftCheck;
    int trackLifterUp = 0;
    boolean nearMountainFlag = false;
    boolean redAlliance = false;


    public CCHS4507Autonomous() {
    }


    int centimetersToCounts(double centimeters) {
        return (int)(centimeters * (countsPerMeter / 100.0));
    }

    double countsToCentimeters(int counts) {
        return (double)counts * (100.0 / countsPerMeter);
    }

    int degreesToCounts(double degrees) {
        return (int)(degrees * (countsPerDonut / 360.0));
    }

    void moveStraight(double distanceCM, double targetSpeed) {
        if (distanceCM > 150.0) {
            noSquiggle = false;
        } else {
            noSquiggle = true;
        }
        speed = targetSpeed;
        if (distanceCM > 0.0) {
            movingForward = true;
        } else {
            movingForward = false;
        }
        leftTargetPosition = motorLeft.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorLeft.setTargetPosition(leftTargetPosition);
        rightTargetPosition = motorRight.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorRight.setTargetPosition(rightTargetPosition);
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
        leftTargetPosition = motorLeft.getCurrentPosition() + degreesToCounts(degrees + gyroError);
        motorLeft.setTargetPosition(leftTargetPosition);
        rightTargetPosition = motorRight.getCurrentPosition() - degreesToCounts(degrees + gyroError);
        motorRight.setTargetPosition(rightTargetPosition);
        motorLeft.setPower(targetSpeed);
        motorRight.setPower(targetSpeed);
    }
    void moveLifter (double degrees) {
        if (degrees == 0.0) {
            stowLifter = true;
        } else {
            stowLifter = false;
        }
            trackLifter.setTargetPosition(trackLifterUp + (int)(degrees * trackLifterCountsPerDegree));
    }

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");
        //servos
        //servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoDist = hardwareMap.servo.get("servoDist");
        climberTriggerLeft = hardwareMap.servo.get("trigLeft");
        climberTriggerRight = hardwareMap.servo.get("trigRight");
        //climberDoor = hardwareMap.servo.get("climberDoor");
        cowCatcher = hardwareMap.servo.get("cowCatcher");
        armLock = hardwareMap.servo.get("armLock");
        trackLock = hardwareMap.servo.get("trackLock");
        colorSense = hardwareMap.colorSensor.get("color");
        //colorGroundSense = hardwareMap.colorSensor.get("colorGround");
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        toMountainSwitch = hardwareMap.digitalChannel.get("mntSw");
        fourthTileSwitch = hardwareMap.digitalChannel.get("fourthTileSw");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        liftCheck =  hardwareMap.touchSensor.get("liftCheck");
        delayPot = hardwareMap.analogInput.get("delayPot");
        moveDelayTime = (long)((delayPot.getValue() * 10000) / 1024);
        nearMountainFlag = nearMountainSwitch.getState();
        fourthTileFlag = fourthTileSwitch.getState();
        toMountainFlag = toMountainSwitch.getState();
        if (redBlueSwitch.getState()) { //This is for when we're going to blue
            redAlliance = false;
            lookingForRedFlag = false;
            lookingForBlueFlag = false;
            servoDist.setPosition(0.75);
        } else { //This is for red
            redAlliance = true;
            lookingForRedFlag = false;
            lookingForBlueFlag = false;
            servoDist.setPosition(0.25);
        }

        if (liftCheck.isPressed()) {
            trackLifterUp = trackLifter.getCurrentPosition();
            trackLifter.setTargetPosition(trackLifterUp);
        }
        distanceToWall = 0.0;
        colorSense.enableLed(false);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        if (toMountainFlag == true) {
            nextMove = MoveState.STRAIGHTTORAMP;
        } else {
            nextMove = MoveState.FIRSTMOVE;
        }
        currentMove = MoveState.MOVEDELAY;
        telemetryMove = MoveState.MOVEDELAY;
        sawRedFlag = false;
        sawBlueFlag = false;
        ambientRed = colorSense.red();
        ambientBlue = colorSense.blue();
        lightAverageCounter = 1;
        desiredHeading = 0;
        speed = 0;
        movingForward = true;
        takeADump = false;
        stowLifter = true;
        fastSpeed = 0.50;
        mediumSpeed = 0.50;
        slowSpeed = 0.35;
        turnSpeed = 0.35;
        delayMillisec = 100;
        moveDoneDelta = centimetersToCounts(0.5);
        xHeading = 0;
        yHeading = 0;
        gyroReadLast = System.currentTimeMillis();
        trackLifter.setPower(0.2);
        trackLifter.setTargetPosition(30);
        servoClimberDumper.setPosition(0.1);
        climberTriggerLeft.setPosition(0.9);
        climberTriggerRight.setPosition(0.1);
        cowCatcher.setPosition(0.2);
        armLock.setPosition(0.5);
        trackLock.setPosition(0.4);
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
        double distance = 0.0;

        if (gyroSense.isCalibrating()) {
            return;
        }
        now = System.currentTimeMillis();
        xHeading = xHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawX();
        yHeading = yHeading + ((double)(now - gyroReadLast) / 1000.0) * (double)gyroSense.rawY();
        gyroReadLast = now;

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
                motorRight.setPower(Range.clip(speed - (gyroError * 0.05), -1.0, 1.0));
                motorLeft.setPower(Range.clip(speed + (gyroError * 0.05), -1.0, 1.0));
                if (lookingForBlueFlag && colorSense.blue() > ambientBlue) {
                    sawBlueFlag = true;
                }
                if (lookingForRedFlag && colorSense.red() > ambientRed) {
                    sawRedFlag = true;
                }
                if (lookingForRedFlag && (colorSense.red() > ambientRed))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    takeADump = true;
                    currentMove = MoveState.MOVEDELAY;
                }
                if (lookingForBlueFlag && (colorSense.blue() > ambientBlue))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    takeADump = true;
                    currentMove = MoveState.MOVEDELAY;
                }
                if (!noSquiggle) {
                    if ((Math.abs(leftTargetPosition - motorLeft.getCurrentPosition()) < moveDoneDelta) ||
                            (Math.abs(rightTargetPosition - motorRight.getCurrentPosition()) < moveDoneDelta)) {
                        motorRight.setPower(0.0);
                        motorLeft.setPower(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    }
                } else {
                    if ((Math.abs(leftTargetPosition - motorLeft.getCurrentPosition()) < moveDoneDelta) &&
                            (Math.abs(rightTargetPosition - motorRight.getCurrentPosition()) < moveDoneDelta)) {
                        currentMove = MoveState.MOVEDELAY;
                    }
                }
                break;

            case STARTTURN:
                if (motorLeft.isBusy() && motorRight.isBusy()) {
                    currentMove = MoveState.TURNING;
                }
                break;

            case TURNING:
                if ((Math.abs(leftTargetPosition - motorLeft.getCurrentPosition()) < moveDoneDelta) &&
                        (Math.abs(rightTargetPosition - motorRight.getCurrentPosition()) < moveDoneDelta)) {
                    currentMove = MoveState.MOVEDELAY;
                }
                break;

            case MOVEDELAY:
                now = System.currentTimeMillis();
                delayUntil = now + moveDelayTime;
                currentMove = MoveState.DELAY;
                break;

            case DELAY:
                if (System.currentTimeMillis() >= delayUntil) {
                    currentMove = nextMove;
                }
                break;

            case FIRSTMOVE:
                moveStraight(60.0 + (fourthTileFlag ? 45.0 : 0.0), fastSpeed);
                cowCatcher.setPosition(0.2);
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
                moveDelayTime = delayMillisec * 4;
                break;

            case FINDWALL:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 20.0) && (distanceToWall <= 120.0)) {
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
                    moveTurn(45.0, turnSpeed);
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
                    lookingForRedFlag = true;
                    lookingForBlueFlag = true;
                } else {
                    moveStraight(90.0, fastSpeed);
                    lookingForBlueFlag = true;
                    lookingForRedFlag = true;
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CENTERBUCKET;
                 //TODO: Verify MEASUREAMBIENT works correctly and enable it.
                nextMove = MoveState.MEASUREAMBIENT;
                telemetryMove = MoveState.FINDBEACON;
                moveDelayTime = delayMillisec;
                break;

            case MEASUREAMBIENT:
                if (lightAverageCounter < 8) {
                    ambientBlue = ambientBlue + colorSense.blue();
                    ambientRed = ambientRed + colorSense.red();
                    lightAverageCounter++;
                    Log.i("ambientRed", Integer.toString(ambientRed));
                    Log.i("ambientBlue", Integer.toString(ambientBlue));
                } else if (lightAverageCounter == 8) {
                    ambientBlue = (int)(((double)ambientBlue / 8.0) + 0.5);
                    ambientRed = (int)(((double)ambientRed / 8.0) + 0.5);
                    lightAverageCounter++;
                    Log.i("ambientRed", Integer.toString(ambientRed));
                    Log.i("ambientBlue", Integer.toString(ambientBlue));
                } else {
                    currentMove = MoveState.CENTERBUCKET;
                }
                telemetryMove = MoveState.MEASUREAMBIENT;
                break;

            case MOVETOBUTTON:
                if (sawRedFlag) {
                    if (redAlliance){
                        moveStraight (-30, mediumSpeed);
                    } else {
                        moveStraight (-5, mediumSpeed);
                    }
                } else if (sawBlueFlag) {
                    if (redAlliance) {
                        moveStraight(-5, mediumSpeed);
                    } else {
                        moveStraight(-30, mediumSpeed);
                    }
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.PRESSBUTTON;
                telemetryMove = MoveState.MOVETOBUTTON;
                moveDelayTime = delayMillisec;
                break;

            case PRESSBUTTON:
                climberTriggerLeft.setPosition(0.8);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CENTERBUCKET;
                telemetryMove = MoveState.PRESSBUTTON;
                moveDelayTime = delayMillisec;
                break;

            case CENTERBUCKET:
                lookingForRedFlag = false;
                lookingForBlueFlag = false;
                climberTriggerLeft.setPosition(0.5);
                // Checking to see whether or not we saw the beacon, and if we didn't don't try
                // and dump the climbers.
                if (!takeADump)  {
                    currentMove = MoveState.ROTATEFROMBEACON;
                } else {
                    if (redAlliance) {
                        moveStraight(-10.0, fastSpeed);
                    } else {
                        moveStraight(10.0, fastSpeed);
                    }
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DUMPTRUCK;
                    telemetryMove = MoveState.CENTERBUCKET;
                    moveDelayTime = delayMillisec;

                }
                break;

            case DUMPTRUCK:
                servoClimberDumper.setPosition(1.0);
                moveDelayTime = 1000;
                nextMove = MoveState.ROTATEFROMBEACON;
                currentMove = MoveState.MOVEDELAY;
                telemetryMove = MoveState.DUMPTRUCK;
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
                    distance = -110.0;
                } else {                // blue alliance
                    distance = -96.0;
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
                        moveTurn(90.0, turnSpeed);
                    } else {
                        moveTurn(-90.0, turnSpeed);
                    }
                } else {
                    if (redAlliance) {
                        moveTurn(-90.0, turnSpeed);
                    } else {
                        moveTurn(90.0, turnSpeed);
                    }
                }
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DOWNTRACK;
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
                moveTurn(180.0, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DOWNTRACK;
                telemetryMove = MoveState.ALIGNRAMP;
                break;

            case STRAIGHTTORAMP:
                moveStraight(190.0, mediumSpeed);
                //cowCatcher.setPosition(0.5);
                moveDelayTime = delayMillisec;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.STRAIGHTTORAMPTURN;
                telemetryMove = MoveState.STRAIGHTTORAMP;
                break;

            case STRAIGHTTORAMPTURN:
                if (redAlliance) {
                    moveTurn(90.0, turnSpeed);
                } else {
                    moveTurn(-90.0, turnSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DOWNTRACK;
                telemetryMove = MoveState.STRAIGHTTORAMPTURN;
                break;

            case DOWNTRACK:
                trackLock.setPosition(0.8);
                moveLifter(90.0);
                cowCatcher.setPosition(.75);
                moveDelayTime = 2000;
                currentMove = MoveState.MOVEDELAY;
                nextMove = MoveState.UPRAMP;
                telemetryMove = MoveState.DOWNTRACK;
                break;

            case UPRAMP:
                if (toMountainFlag == false) {
                    moveStraight(-100.0, fastSpeed);
                } else {
                    moveStraight(-160.0, fastSpeed);
                }
                trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                trackLifter.setPower(0.0);
                trackLifter.setPowerFloat();
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.UPRAMP;
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                telemetryMove = MoveState.DONE;
                break;
        }
        if (stowLifter && !liftCheck.isPressed() && !trackLifter.isBusy()) {
            trackLifter.setTargetPosition(trackLifter.getCurrentPosition() - (int)trackLifterCountsPerDegree);
        }
        if (liftCheck.isPressed()) {
            trackLifterUp = trackLifter.getCurrentPosition();
        }
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("motorLeft", Integer.toString(Math.abs(motorLeft.getCurrentPosition() - leftTargetPosition)));
        telemetry.addData("motorRight", Integer.toString(Math.abs(motorLeft.getCurrentPosition() - leftTargetPosition)));
        telemetry.addData("desiredHeading", Integer.toString(desiredHeading));
        telemetry.addData("gyro", Integer.toString(gyroSense.getHeading()));
        telemetry.addData("distToWall", distanceToWall);
//        telemetry.addData("Xheading", xHeading);
//        telemetry.addData("yHeading", yHeading);
//        telemetry.addData("POT", Long.toString(moveDelayTime));

//        Log.i("Current Move", currentMove.toString());
//        Log.i("desiredHeading", Integer.toString(desiredHeading));
//        Log.i("gyro", Integer.toString(gyroSense.getHeading()));
//        Log.i("colorRed", Integer.toString(colorSense.red()));
//        Log.i("colorBlue", Integer.toString(colorSense.blue()));
//        Log.i("colorGreen", Integer.toString(colorSense.green()));
//        Log.i("colorAlpha", Integer.toString(colorSense.alpha()));
//        Log.i("ambientRed", Integer.toString(ambientRed));
//        Log.i("ambientBlue", Integer.toString(ambientBlue));
    }

    @Override
    public void stop() {
    }
}

































// ;)