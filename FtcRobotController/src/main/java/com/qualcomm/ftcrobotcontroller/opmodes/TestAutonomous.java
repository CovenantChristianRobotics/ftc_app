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
        START1, DELAY, DELAY1, DELAY2, STARTMOVE, MOVING, START2, START3, FINDRED, DONE
    }

    enum BeaconState {
        NOT_LOOKING, LOOKING_START, LOOKING_END, DONE
    }

//    enum BeaconColor {
//        RED, BLUE
//    }
//
//    enum RobotSide {
//        LEFT, RIGHT
//    }

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    ColorSensor ColorSense;
    //GyroSensor Gyro;
    //IrSeekerSensor IrSense;
    OpticalDistanceSensor OpticalDistance;
    MoveState currentMove;
    MoveState nextMove;
    BeaconState redState;
    //double[][][] beaconLocation;
    int redStartRight;
    int redStartLeft;
    int redEndRight;
    int redEndLeft;
    BeaconState blueState;
    int blueStartRight;
    int blueStartLeft;
    int blueEndRight;
    int blueEndLeft;
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
        return (((double)counts / encoderCounts) * (wheelDiameter * Math.PI));
    }

    int degreesToCounts(double degrees) {
        double wheelBase = 40.3225; // wheelbase of the primary drive wheels
        double oneDegree = ((wheelBase * Math.PI) / 360);   // calculates the distance of one degree
        return centimetersToCounts(oneDegree * degrees);
    }

    void moveStraight(int distanceCM, double speed) {
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

        leftTarget = motorLeft.getCurrentPosition() + degreesToCounts(degrees);
        motorLeft.setTargetPosition(leftTarget);
        rightTarget = motorRight.getCurrentPosition() - degreesToCounts(degrees);
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
        //Gyro = hardwareMap.gyroSensor.get("GyroSense");
        OpticalDistance = hardwareMap.opticalDistanceSensor.get("opDistance");
        //IrSense = hardwareMap.irSeekerSensor.get("IRSense");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        currentMove = MoveState.START1;
        redState = BeaconState.NOT_LOOKING;
        blueState = BeaconState.NOT_LOOKING;
        //beaconLocation = new double[2][2][2];
    }

    @Override
    public void loop() {

        switch (currentMove) {
            case START1:
                moveStraight(80, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY1;
                break;

            case DELAY1:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START2;
                break;

            case DELAY:
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentMove = nextMove;
                }
                break;

            case STARTMOVE:
                if (motorLeft.isBusy() && motorRight.isBusy()) {
                    currentMove = MoveState.MOVING;
                }
                break;

            case MOVING:
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    currentMove = nextMove;
                }
                break;

            case START2:
                moveTurn(90.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY2;
                break;

            case DELAY2:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START3;
                break;

            case START3:
                moveStraight(160, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.FINDRED;
                redState = BeaconState.LOOKING_START;
                blueState = BeaconState.LOOKING_START;
                break;

            case FINDRED:
                moveStraight((int)countsToCentimeters(-(motorLeft.getCurrentPosition() - ((redStartLeft + redEndLeft) / 2))), 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                break;
        }
        switch (redState) {
            case NOT_LOOKING:
                break;
            case LOOKING_START:
                if (ColorSense.red() >= 1) {
                    redStartLeft = motorLeft.getCurrentPosition();
                    redStartRight = motorRight.getCurrentPosition();
                    redState = BeaconState.LOOKING_END;
                }
                break;
            case LOOKING_END:
                if (ColorSense.red() == 0) {
                    redEndLeft = motorLeft.getCurrentPosition();
                    redEndRight = motorRight.getCurrentPosition();
                    redState = BeaconState.DONE;
                }
                break;
            case DONE:
                break;
        }
        switch (blueState) {
            case NOT_LOOKING:
                break;
            case LOOKING_START:
                if (ColorSense.blue() >= 1) {
                    blueStartLeft = motorLeft.getCurrentPosition();
                    blueStartRight = motorRight.getCurrentPosition();
                    blueState = BeaconState.LOOKING_END;
                }
                break;
            case LOOKING_END:
                if (ColorSense.blue() == 0) {
                    blueEndLeft = motorLeft.getCurrentPosition();
                    blueEndRight = motorRight.getCurrentPosition();
                    blueState = BeaconState.DONE;
                }
                break;
            case DONE:
                break;
        }
        if (gamepad1.start) {
            currentMove = MoveState.START1;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("RedStart", redStartLeft);
        telemetry.addData("RedEnd", redEndLeft);
        telemetry.addData("BlueStart", blueStartLeft);
        telemetry.addData("BlueEnd", blueEndLeft);
        //telemetry.addData("GyroSense", (float) Gyro.getRotation());
        telemetry.addData("OpticalSense", (float) OpticalDistance.getLightDetectedRaw());
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}