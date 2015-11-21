package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import java.util.Date;
/**
 * Created by cchsrobochargers on 11/14/15.
 */
public class TestAutonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, DONE
    }

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    ColorSensor ColorSense;
    MoveState currentMove;
    MoveState nextMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    boolean lookingForWallFlag;
    long delayUntil;
    Date now;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;

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
        ColorSense = hardwareMap.colorSensor.get("color");
        ColorSense.enableLed(true);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        currentMove = MoveState.FIRSTMOVE;
        lookingForRedFlag = false;
        lookingForWallFlag = false;
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
    }

    @Override
    public void loop() {
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
                if (lookingForRedFlag && (ColorSense.red() >= 1))  {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    currentMove = MoveState.MOVEDELAY;
                }
                if (lookingForWallFlag &&
                        (ultraSense.getUltrasonicLevel() > 15) && (ultraSense.getUltrasonicLevel() <= 20)) {
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
                moveStraight(80.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                moveDelayTime = 100;
                break;

            case TURNDIAG:
                moveTurn(45.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVEDIAG;
                moveDelayTime = 100;
                break;

            case MOVEDIAG:
                moveStraight(259.0, 0.5);
                lookingForWallFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNALONGWALL;
                moveDelayTime = 100;
                break;

            case FINDWALL:
                moveStraight(40.0, 0.5);
                lookingForWallFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNALONGWALL;
                moveDelayTime = 1000;
                break;

            case TURNALONGWALL:
                moveTurn(-45.0, 0.5);
                lookingForWallFlag = false;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDBEACON;
                moveDelayTime = 100;
                break;

            case FINDBEACON:
                moveStraight(-122.0, 0.5);
                lookingForRedFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.ROTATEFROMBEACON;
                moveDelayTime = 2000;
                break;

            case ROTATEFROMBEACON:
                moveTurn(50.0, 0.5);
                lookingForRedFlag = false;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.MOVETORAMP;
                moveDelayTime = 100;
                break;

            case MOVETORAMP:
                moveStraight(-103.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTORAMP;
                moveDelayTime = 100;
                break;

            case TURNTORAMP:
                moveTurn(-101.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
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

