package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import java.util.Date;
/**
 * Created by cchsrobochargers on 11/14/15.
 */
public class TestAutonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, DUMPTRUCK, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, STOPATRAMP, UPRAMP, DONE
    }

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servo1;
    ColorSensor ColorSense;
    MoveState currentMove;
    MoveState nextMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    boolean sawBlueFlag;
    long delayUntil;
    double speed;
    double slowSpeed;
    double turnSpeed;
    long delay;
    Date now;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;

    // Switches
    boolean nearMountainFlag = false;


    public TestAutonomous() {
    }

    int centimetersToCounts(double centimeters) {
        double wheelDiameter = 10.1;    // wheel diameter in cm
        double encoderCounts = 1120.0;  // encoder counts per revolution of the drive train motors
        return (int) ((centimeters / (wheelDiameter * Math.PI)) * encoderCounts);
    }

    double countsToCentimeters(int counts) {
        double wheelDiameter = 10.1;
        double encoderCounts = 1120.0;
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
        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        //servos
        servoBeaconPinion = hardwareMap.servo.get("beacon_pinion");
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servo1 = hardwareMap.servo.get("servo_1");
        ColorSense = hardwareMap.colorSensor.get("color");
        ColorSense.enableLed(true);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        currentMove = MoveState.FIRSTMOVE;
        lookingForRedFlag = false;
        sawBlueFlag = false;
        speed = 0.5;
        slowSpeed = 0.3;
        turnSpeed = 0.5;
        delay = 100;
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        servoBeaconPinion.setPosition(0.0);
        servoClimberDumper.setPosition(0.9);
        servoBeaconPusher.setPosition(1.0);
        servo1.setPosition(0.25);
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        gyroSense.calibrate();
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
                moveDelayTime = delay;
                break;

            case TURNDIAG:
                moveTurn(45.0, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVEDIAG;
                moveDelayTime = delay;
                break;

            case MOVEDIAG:
                moveStraight(219.0, speed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDWALL;
                moveDelayTime = 1000;
                break;

            case FINDWALL:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                    moveStraight((distanceToWall - 23.0) * 1.414, slowSpeed);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNALONGWALL;
                    moveDelayTime = delay;
                }
                break;

            case TURNALONGWALL:
                moveTurn(-45.0, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDBEACON;
                moveDelayTime = delay;
                break;

            case FINDBEACON:
                moveStraight(-122.0, speed);
                lookingForRedFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DUMPTRUCK;
                moveDelayTime = delay;
                break;

            case DUMPTRUCK:
                servoClimberDumper.setPosition(0.25);
                currentMove = MoveState.MOVEDELAY;
                nextMove = MoveState.ROTATEFROMBEACON;
                moveDelayTime = 1000;
                break;

            case ROTATEFROMBEACON:
                moveTurn(50.0, turnSpeed);
                lookingForRedFlag = false;
                servoClimberDumper.setPosition(1.0);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVETORAMP;
                moveDelayTime = delay;
                break;

            case MOVETORAMP:
                double distance = -91.0;
                if (!sawBlueFlag) {
                    distance -= 10.0;
                }
                if (!nearMountainFlag) {
                    distance -= 67.0;
                }
                moveStraight(distance, speed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTORAMP;
                moveDelayTime = delay;
                break;

            case TURNTORAMP:
                if (nearMountainFlag) {
                    moveTurn(91.0, turnSpeed);
                } else {
                    moveTurn(-93.0, turnSpeed);
                }
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.STOPATRAMP;
                moveDelayTime = delay;
                break;

            case STOPATRAMP:
                if (nearMountainFlag) {
                    moveStraight(40.0, speed);
                } else {
                    moveStraight(200.0, speed);
                }
                servo1.setPosition(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.UPRAMP;
                break;

            case UPRAMP:
                distanceToWall = ultraSense.getUltrasonicLevel();
                if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                    moveStraight(distanceToWall - 5.0, slowSpeed);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DONE;
                }
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                break;
        }

        if (gamepad1.start) {
            currentMove = MoveState.FIRSTMOVE;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Color", (float) ColorSense.red());
        telemetry.addData("gyro", (float) gyroSense.getHeading());
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
    }

    @Override
    public void stop() {
    }
}

