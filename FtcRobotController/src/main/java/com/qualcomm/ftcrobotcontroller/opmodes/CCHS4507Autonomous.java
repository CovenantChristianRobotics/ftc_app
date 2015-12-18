package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import java.util.Date;
/**
 * Created by cchsrobochargers on 12/17/15.
 */
public class CCHS4507Autonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, DUMPTRUCK, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, STOPATRAMP, UPRAMP, DONE
    }

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;
    //servos
    Servo servoBeaconPusher;
    Servo servoClimberDumper;Servo servoDist;
    ColorSensor ColorSense;
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
    double slowSpeed;
    double turnSpeed;
    long delay;
    Date now;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;

    // robot constants
    double wheelDiameter = 6.0 / 2.0;   // wheel diameter in cm 2 to 1 gear ratio
    double encoderCounts = 1120.0;      // encoder counts per revolution of the drive train motors
    double wheelBase = 39.8;            // wheelbase of the primary drive wheels

    // Switches
    DigitalChannel nearMountainSwitch;
    DigitalChannel redBlueSwitch;
    // DigitalChannel delaySwitch;
    // DigitalChannel tileSwitch

    boolean nearMountainFlag = false;
    double redBlueFlag = 1.0;
    // long delayTimeFlag = 10;
    

    public CCHS4507Autonomous() {
    }

    int centimetersToCounts(double centimeters) {
        return (int) ((centimeters / (wheelDiameter * Math.PI)) * encoderCounts);
    }

    double countsToCentimeters(int counts) {
        return (((double) counts / encoderCounts) * (wheelDiameter * Math.PI));
    }

    int degreesToCounts(double degrees) {
        double wheelBase = 40.3225; // wheelbase of the primary drive wheels
        double oneDegree = ((wheelBase * Math.PI) / 360);   // calculates the distance of one degree
        return centimetersToCounts(oneDegree * degrees);
    }

    void moveStraight(double distanceCM, double speed) {
        int rightTarget;
        int leftTarget;
        leftTarget = motorLeft.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorLeft.setTargetPosition(leftTarget);
        rightTarget = motorRight.getCurrentPosition() + centimetersToCounts(distanceCM);
        motorRight.setTargetPosition(rightTarget);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
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

        leftTarget = motorLeft.getCurrentPosition() - degreesToCounts(degrees);
        motorLeft.setTargetPosition(leftTarget);
        rightTarget = motorRight.getCurrentPosition() + degreesToCounts(degrees);
        motorRight.setTargetPosition(rightTarget);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
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
        nearMountainSwitch = hardwareMap.digitalChannel.get("nearMtnSw");
        redBlueSwitch = hardwareMap.digitalChannel.get("rbSw");
        // delaySwitch = hardwareMap.digitalChannel.get("dSw")
        // tileSwitch = hardwareMap.digitalChannel.get("tSw")
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        nearMountainFlag = nearMountainSwitch.getState();
        if (redBlueSwitch.getState()) {
            redBlueFlag = 1.0;
            lookingForRedFlag = true;
            lookingForBlueFlag = false;
            servoDist.setPosition(0.75);
        } else {
            redBlueFlag = -1.0;
            lookingForRedFlag = false;
            lookingForBlueFlag = true;
            servoDist.setPosition(0.25);
        }
        ColorSense.enableLed(true);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        trackLifter.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        currentMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        sawRedFlag = false;
        sawBlueFlag = false;
        speed = 1.0;
        slowSpeed = 0.75;
        turnSpeed = 0.75;
        delay = 100;
        trackLifter.setPower(0.1);
        trackLifter.setTargetPosition(30);
        servoClimberDumper.setPosition(0.9);
        servoBeaconPusher.setPosition(0.0);
        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
    }

    @Override
    public void loop() {
        double distanceToWall;
        int dumperCounter = 0;
        int dumperCounterThresh = 30;
        double dumperPosition = 0.9;
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
                if (ColorSense.blue() >= 1) {
                    sawBlueFlag = true;
                }
                if (lookingForRedFlag && (ColorSense.red() >= 1))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
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
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentMove = nextMove;
                }
                break;

            case FIRSTMOVE:
                moveStraight(80.0, speed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = delay;
                break;

            case TURNDIAG:
                moveTurn(45.0 * redBlueFlag, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVEDIAG;
                telemetryMove = MoveState.TURNDIAG;
                moveDelayTime = delay;
                break;

            case MOVEDIAG:
                moveStraight(219.0, speed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDWALL;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = 1000;
                break;

            case FINDWALL:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 40.0) && (distanceToWall <= 80.0)) {
                    moveStraight((distanceToWall - 33.0) * 1.414, slowSpeed);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNALONGWALL;
                    telemetryMove = MoveState.FINDWALL;
                    moveDelayTime = delay;
                }
                break;

            case TURNALONGWALL:
                moveTurn(-45.0 * redBlueFlag, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDBEACON;
                telemetryMove = MoveState.TURNALONGWALL;
                moveDelayTime = delay;
                break;

            case FINDBEACON:
                moveStraight(-122.0, speed);
                lookingForRedFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DUMPTRUCK;
                telemetryMove = MoveState.FINDBEACON;
                moveDelayTime = delay;
                break;

            case DUMPTRUCK:
                // If timer hits threshold, reset timer and move servo
                if (dumperCounter >= dumperCounterThresh) {
                    dumperPosition -= .02;
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
                dumperCounter ++;
                break;

            case ROTATEFROMBEACON:
                moveTurn(50.0 * redBlueFlag, turnSpeed);
                lookingForRedFlag = false;
                servoClimberDumper.setPosition(1.0);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVETORAMP;
                telemetryMove = MoveState.ROTATEFROMBEACON;
                moveDelayTime = delay;
                break;

            case MOVETORAMP:
                double distance = -71.0;
                if (!sawBlueFlag) {
                    distance -= 10.0;
                }
                if (!nearMountainFlag) {
                    distance -= 67.0;
                }
                moveStraight(distance, speed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTORAMP;
                telemetryMove = MoveState.MOVETORAMP;
                moveDelayTime = delay;
                break;

            case TURNTORAMP:
                if (nearMountainFlag) {
                    moveTurn(91.0 * redBlueFlag, turnSpeed);
                } else {
                    moveTurn(-98.0 * redBlueFlag, turnSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.STOPATRAMP;
                telemetryMove = MoveState.TURNTORAMP;
                moveDelayTime = delay;
                break;

            case STOPATRAMP:
                if (nearMountainFlag) {
                    moveStraight(40.0, speed);
                } else {
                    moveStraight(200.0, speed);
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

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("Color", (float) ColorSense.red());
        telemetry.addData("gyro", (float) gyroSense.getHeading());
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
    }

    @Override
    public void stop() {
    }
}

