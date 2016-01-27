package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Date;

/**
 * Created by cchsrobochargers on 11/14/15.
 */
public class SpareParts5256Autonomous extends OpMode {
    enum MoveState {
        STARTMOVE, MOVING, MOVEDELAY, DELAY, FIRSTMOVE, TURNDIAG, MOVEDIAGNEAR, MOVEDIAGFAR,
        TURNTONEARMOUNTAIN, TURNTOFARMOUNTAIN, DRIVETONEARMOUNTAIN, DRIVETOFARMOUNTAIN, CLIMBMOUNTAIN, DONE
    }

    //dc motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    //    DcMotor chinUp;
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
    int gyroError;
    int desiredHeading;
    UltrasonicSensor ultraSense;
    TouchSensor beaconPinionStop;
    //    TouchSensor leftWheelStop;
//    TouchSensor rightWheelStop;
    TouchSensor beaconPinionIn;
    TouchSensor beaconPinionOut;
    //Statemachine options
    MoveState currentMove;
    MoveState nextMove;
    MoveState mountainTurn;
    MoveState telemetryMove;
    double ifRedOnBeacon;
    double ifBlueOnBeacon;
    boolean lookingForFlag;
    boolean dumpedClimbers;
    boolean pushedButton;
    double speed;
    boolean movingForward;
    double fastSpeed = 0.75;
    double slowSpeed = 0.25;
    double turnSpeed = 0.6;
    // Switches
    DigitalChannel nearMtnSwitch;
    DigitalChannel redBlueBeaconSwitch;
    DigitalChannel delayPotSwitch;
    //    DigitalChannel tileSwitch;
    AnalogInput delayPotentiometer;
    int nearMtn;
    int redBlue;
    double mountainDist;
    double upMountainDist;
    int delay = 101;
    long delayTime;
    boolean redAlliance = false;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    boolean sawBlueFlag;
    boolean sawRedFlag;
    int dumperCounter = 0;
    double dumperPosition = 0.9;
    boolean nearMountainFlag = false;
    //    int tile;
    ElapsedTime matchTime;

    //delay settings
    long delayUntil;
    long moveDelayTime;
    Date now;
    double countsPerDonut = 6083.0;    // Encoder counts per 360 degrees
    double countsPerMeter = 5076.0;    // Found this experimentally: Measured one meter, drove distance, read counts
    int dumperCounterThresh = 8;       // Doesn't let the dumper counter get above a certain number


    public SpareParts5256Autonomous() {
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

        leftTarget = leftDrive.getCurrentPosition() - degreesToCounts(degrees);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + degreesToCounts(degrees);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    @Override
    public void init() {
        //dc Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
//        chinUp = hardwareMap.dcMotor.get("chinUp");
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
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        gyroSense.calibrate();
        ultraSense = hardwareMap.ultrasonicSensor.get("fUltraSense");
//        beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
//        leftWheelStop = hardwareMap.touchSensor.get("lWStop");
//        rightWheelStop = hardwareMap.touchSensor.get("rWStop");
        beaconPinionIn = hardwareMap.touchSensor.get("bPIn");
        beaconPinionOut = hardwareMap.touchSensor.get("bPOut");
        //motor configurations
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //statemachine settings
        currentMove = MoveState.FIRSTMOVE;
        nextMove = MoveState.TURNDIAG;
        telemetryMove = MoveState.FIRSTMOVE;
        lookingForFlag = false;
        // switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
//        delayPotSwitch = hardwareMap.digitalChannel.get("dSw");
        delayPotentiometer = hardwareMap.analogInput.get("dP");
        delayTime = (long) (delayPotentiometer.getValue() * (10000 / 1024));
        matchTime = new ElapsedTime();
        //servo positions
        servoBeaconPinion.setPosition(0.0);
        servoClimberDumper.setPosition(1.0);
        servoBeaconPusher.setPosition(0.3);
        servoUltraSense.setPosition(0.75);
        moveDelayTime = delayTime;
        dumpedClimbers = false;
        pushedButton = false;
        if (redBlueBeaconSwitch.getState()) { //red alliance
            redBlue = 1;
            servoUltraSense.setPosition(0.25);
        } else { // blue alliance
            redBlue = -1;
            servoUltraSense.setPosition(0.75);
        }
        if (redBlueBeaconSwitch.getState()) { //This is for when we're going to blue
            redAlliance = false;
            lookingForRedFlag = false;
            lookingForBlueFlag = true;
            servoUltraSense.setPosition(0.25);
        } else { //This is for red
            redAlliance = true;
            lookingForRedFlag = true;
            lookingForBlueFlag = false;
            servoUltraSense.setPosition(0.75);
        }

        if (nearMtnSwitch.getState()) {
            mountainTurn = MoveState.MOVEDIAGNEAR;
            mountainDist = 100.0;
        } else {
            mountainTurn = MoveState.MOVEDIAGFAR;
            mountainDist = 200.0;
        }


        // align color sensor
//        while (beaconPinionIn.isPressed() == false) {
//            servoBeaconPinion.setPosition(1.0);
//        }
//        servoBeaconPinion.setPosition(0.5);
//
        //  while (!beaconPinionStop.isPressed()) {
        //      servoBeaconPinion.setPosition(0.5);
        //  }
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
            case STARTMOVE:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.MOVING;
                }
                break;

            case MOVING:
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
                moveStraight(91.44, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = delay;
                break;

            case TURNDIAG:
                moveTurn(-45.0 * redBlue, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = mountainTurn;
                telemetryMove = MoveState.TURNDIAG;
                moveDelayTime = delay;
                break;

            case MOVEDIAGNEAR:
                moveStraight(121.92, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTONEARMOUNTAIN;
                telemetryMove = MoveState.MOVEDIAGNEAR;
                moveDelayTime = delay;
                break;

            case MOVEDIAGFAR:
                moveStraight(60.96, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTOFARMOUNTAIN;
                telemetryMove = MoveState.MOVEDIAGFAR;
                moveDelayTime = delay;
                break;

            case TURNTONEARMOUNTAIN:
                moveTurn(-90.0 * redBlue, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DRIVETONEARMOUNTAIN;
                telemetryMove = MoveState.TURNTONEARMOUNTAIN;
                moveDelayTime = delay;
                break;

            case TURNTOFARMOUNTAIN:
                moveTurn(-90.0 * redBlue, turnSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DRIVETOFARMOUNTAIN;
                telemetryMove = MoveState.TURNTOFARMOUNTAIN;
                moveDelayTime = delay;
                break;

            case DRIVETONEARMOUNTAIN:
                moveStraight(106.68, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CLIMBMOUNTAIN;
                telemetryMove = MoveState.DRIVETONEARMOUNTAIN;
                moveDelayTime = delay;
                break;

            case DRIVETOFARMOUNTAIN:
                moveStraight(180.34,fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.CLIMBMOUNTAIN;
                telemetryMove = MoveState.DRIVETOFARMOUNTAIN;
                moveDelayTime = delay;
                break;

            case CLIMBMOUNTAIN:
                moveStraight(upMountainDist, fastSpeed);
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.CLIMBMOUNTAIN;
                break;

            case DONE:
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                telemetryMove = MoveState.DONE;
                break;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Color", (float) beaconColorSense.red());
        telemetry.addData("gyro", (float) gyroSense.getHeading());
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
    }

    @Override
    public void stop() {
    }
}