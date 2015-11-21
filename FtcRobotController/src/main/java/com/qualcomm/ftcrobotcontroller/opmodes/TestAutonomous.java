package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import java.util.Date;

/**
 * Created by cchsrobochargers on 11/14/15.
 */
public class TestAutonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, MOVEDELAY, START1, START2, START3, START4, START5,START6,
        START7, START8, DONE
    }

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    ColorSensor ColorSense;
    MoveState currentMove;
    MoveState nextMove;
    long moveDelayTime;
    boolean lookingForRedFlag;
    long delayUntil;
    Date now;

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
        currentMove = MoveState.START1;
        lookingForRedFlag = false;
    }

    @Override
    public void loop() {

        switch (currentMove) {


            case STARTMOVE:
                if (motorLeft.isBusy() && motorRight.isBusy()) {
                    currentMove = MoveState.MOVING;
                }
                break;

            case MOVING:
                if ((ColorSense.red() >= 1) && lookingForRedFlag) {
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

            case START1:
                moveStraight(80.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START2;
                moveDelayTime = 100;
                break;

            case START2:
                moveTurn(45.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START3;
                moveDelayTime = 100;
                break;

            case START3:
                moveStraight(259.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START4;
                moveDelayTime = 100;
                break;

            case START4:
                moveTurn(-45.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START5;
                moveDelayTime = 100;
                break;

            case START5:
                moveStraight(-122.0, 0.5);
                lookingForRedFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START6;
                moveDelayTime = 1000;
                break;

            case START6:
                moveTurn(50.0, 0.5);
                lookingForRedFlag = false;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START7;
                moveDelayTime = 100;
                break;

            case START7:
                moveStraight(-103.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.START8;
                moveDelayTime = 100;
                break;

            case START8:
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
            currentMove = MoveState.START1;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Color", (float) ColorSense.red());
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}

