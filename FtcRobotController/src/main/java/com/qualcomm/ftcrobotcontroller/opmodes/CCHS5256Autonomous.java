package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import java.util.Date;

/**
 * Created by Robotics on 10/7/2015.
 */
public class CCHS5256Autonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, SERVOPINIONREDMOVE, SERVOPINIONNULLMOVE, SERVOPINIONBLUEMOVE,
        SERVOPUSHERMOVE, SERVODUMPERMOVE, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        DRIVEALONGWALL, FINDBEACON, ALIGNDUMPER, DUMPCLIMBERS, ALIGNPRESSER, PRESSBUTTON,
        PULLAHEADALONGWALL, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, UPRAMP, DONE
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
    DcMotor chinUp;
    //servo controllers
    ServoController beaconController;
    ServoController alignmentController;
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    //Servo servoClimberDumper;
    Servo servoUltraSense;
    Servo leftOmniPinion;
    Servo rightOmniPinion;
    //sensors
    ColorSensor beaconColorSense;
//    ColorSensor floorColorSense;
//    OpticalDistanceSensor leftWheelAlignment;
//    OpticalDistanceSensor rightWheelAlignment;
    GyroSensor gyroSense;
    int preTurnHeading;
    UltrasonicSensor ultraSense;
    TouchSensor beaconPinionAlignment;
    TouchSensor beaconPinionStop;
    TouchSensor leftWheelStop;
    TouchSensor rightWheelStop;
    //Statemachine options
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    double ifRedOnBeacon;
    double ifBlueOnBeacon;
    boolean lookingForRedFlag;
    boolean sawBlueFlag;
    //delay settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    public CCHS5256Autonomous () {
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

    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees
     * @param speed
     */
    void moveTurnWithGyro(int degrees, double speed) {
        if (gyroSense.getHeading() == (preTurnHeading + degrees)) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        } else if (gyroSense.getHeading() == ((preTurnHeading + degrees) - 360)) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        } else if (gyroSense.getHeading() == ((preTurnHeading + degrees) + 360)) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        } else if (degrees > 0 && gyroSense.getHeading() < (preTurnHeading + degrees)) {
            leftDrive.setPower(-speed);
            rightDrive.setPower(speed);
        } else if (degrees < 0 && gyroSense.getHeading() > (preTurnHeading - degrees)) {
            leftDrive.setPower(speed);
            rightDrive.setPower(-speed);
        } else if (degrees == 0) {
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
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
//        servoClimberDumper.setPosition(climberDumperPosition);
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
        chinUp = hardwareMap.dcMotor.get("chinUp");
        //servo controllers
        beaconController = hardwareMap.servoController.get("beaconCtlr");
        alignmentController = hardwareMap.servoController.get("alignCtlr");
        //servos
        servoBeaconPinion = hardwareMap.servo.get("beaconPinion");
        servoBeaconPusher = hardwareMap.servo.get("beaconPusher");
        //servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoUltraSense = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        //sensors
        beaconColorSense = hardwareMap.colorSensor.get("bColorSense");
        beaconColorSense.enableLed(false);
//        floorColorSense = hardwareMap.colorSensor.get("fColorSense");
//        floorColorSense.enableLed(true);
        gyroSense = hardwareMap.gyroSensor.get("gyro");
//        gyroSense.calibrate();
//        while (gyroSense.isCalibrating()) {
//        }
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        beaconPinionAlignment = hardwareMap.touchSensor.get("bPAlign");
        beaconPinionStop = hardwareMap.touchSensor.get("bPStop");
        leftWheelStop = hardwareMap.touchSensor.get("lWStop");
        rightWheelStop = hardwareMap.touchSensor.get("rWStop");
        //motor configurations
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //statemachine settings
        currentMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        lookingForRedFlag = false;
        sawBlueFlag = false;
        //servo positions
        moveBeaconPinion(0.0);
        moveClimberDump(1.0);
        moveBeaconPress(1.0);
        servoUltraSense.setPosition(0.25);
        // align color sensor
        // while (!beaconPinionStop.isPressed()) {
        //     moveBeaconPinion(0.5);
        // }
        // // align omniwheels
        // while (!leftWheelStop.isPressed()) {
        //     moveLeftOmnipinion(0.5);
        // }
        // while (!rightWheelStop.isPressed()) {
        //     moveRightOmnipinion(0.5);
        // }

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
                    if (lookingForRedFlag && ((beaconColorSense.red() >= 1) || (beaconColorSense.blue() >= 1))) {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    }
                    if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                        currentMove = MoveState.MOVEDELAY;
                    }
                    break;


                case SERVOPINIONNULLMOVE:
                    if (beaconColorSense.red() < 1.0 && beaconColorSense.blue() < 1.0) {
                        moveBeaconPinion(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    } else if (beaconColorSense.red() >= 1.0 || beaconColorSense.blue() >= 1.0) {
                        moveBeaconPinion(beaconPinionPosition);
                        currentMove = MoveState.SERVOPINIONNULLMOVE;
                    }
                    break;

                case SERVOPINIONREDMOVE:
                    if (beaconColorSense.red() >= 1.0) {
                        moveBeaconPinion(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    } else if (beaconColorSense.red() < 1.0) {
                        moveBeaconPinion(beaconPinionPosition);
                        currentMove = MoveState.SERVOPINIONREDMOVE;
                    }

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
                    preTurnHeading = gyroSense.getHeading();
                    moveStraight(80.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNDIAG;
                    telemetryMove = MoveState.FIRSTMOVE;
                    moveDelayTime = 75;
                    break;

                case TURNDIAG:
                    moveTurn(45.0, 0.6);
//                    moveTurnWithGyro(45, 0.5);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.MOVEDIAG;
                    telemetryMove = MoveState.TURNDIAG;
                    moveDelayTime = 75;
                    break;

                case MOVEDIAG:
                    preTurnHeading = gyroSense.getHeading();
                    moveStraight(171.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.FINDWALL;
                    telemetryMove = MoveState.MOVEDIAG;
                    moveDelayTime = 250;
                    break;

                case FINDWALL:
                    distanceToWall = ultraSense.getUltrasonicLevel();
                    if ((distanceToWall > 30.0) && (distanceToWall <= 90.0)) {
                        moveStraight((distanceToWall - 33.0) * 1.414, 0.5);
                        currentMove = MoveState.STARTMOVE;
                        nextMove = MoveState.TURNALONGWALL;
                        telemetryMove = MoveState.FINDWALL;
                        moveDelayTime = 250;
                    }
                    break;

                case TURNALONGWALL:
                    moveTurn(-45.0, 0.6);
//                    moveTurnWithGyro(-45, 0.5);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DRIVEALONGWALL;
                    telemetryMove = MoveState.TURNALONGWALL;
                    moveDelayTime = 75;
                    break;

                case DRIVEALONGWALL:
                    preTurnHeading = gyroSense.getHeading();
                    moveStraight(-50.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.FINDBEACON;
                    telemetryMove = MoveState.DRIVEALONGWALL;
                    moveDelayTime = 75;
                    break;

                case FINDBEACON:
                    moveStraight(-30, 0.20);
                    lookingForRedFlag = true;
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.ALIGNDUMPER;
                    telemetryMove = MoveState.FINDBEACON;
                    moveDelayTime = 500;
                    break;

                case ALIGNDUMPER:
                    ifRedOnBeacon = beaconColorSense.red();
                    ifBlueOnBeacon = beaconColorSense.blue();
                    moveBeaconPinion(0.5);
                    currentMove = MoveState.SERVOPINIONNULLMOVE;
                    nextMove = MoveState.DUMPCLIMBERS;
                    telemetryMove = MoveState.ALIGNDUMPER;
                    moveDelayTime = 3000;
                    break;

                case DUMPCLIMBERS:
                    moveClimberDump(0.0);
                    moveClimberDump(1.0);
                    currentMove = MoveState.MOVEDELAY;
                    nextMove = MoveState.ALIGNPRESSER;
                    telemetryMove = MoveState.DUMPCLIMBERS;
                    moveDelayTime = 3000;
                    break;

                case ALIGNPRESSER:
                    if (ifRedOnBeacon >= 1.0) {
                        moveBeaconPinion(-0.5);
                    } else if (ifRedOnBeacon < 1.0) {
                        moveBeaconPinion(0.5);
                    }
                    currentMove = MoveState.SERVOPINIONREDMOVE;
                    nextMove = MoveState.PRESSBUTTON;
                    telemetryMove = MoveState.ALIGNPRESSER;
                    moveDelayTime = 3000;
                    break;

                case PRESSBUTTON:
                    moveBeaconPress(0.0);
                    moveBeaconPress(1.0);
                    currentMove = MoveState.MOVEDELAY;
                    nextMove = MoveState.PULLAHEADALONGWALL;
                    telemetryMove = MoveState.PRESSBUTTON;
                    moveDelayTime = 3000;
                    break;

                case PULLAHEADALONGWALL:
                    moveStraight(30, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.ROTATEFROMBEACON;
                    telemetryMove = MoveState.PULLAHEADALONGWALL;
                    moveDelayTime = 200;
                    break;

                case ROTATEFROMBEACON:
                    moveTurn(50.0, 0.6);
//                    moveTurnWithGyro(50, 0.6);
                    lookingForRedFlag = false;
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.MOVETORAMP;
                    telemetryMove = MoveState.ROTATEFROMBEACON;
                    moveDelayTime = 75;
                    break;

                case MOVETORAMP:
                    preTurnHeading = gyroSense.getHeading();
                    moveStraight(-103.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.TURNTORAMP;
                    telemetryMove = MoveState.MOVETORAMP;
                    moveDelayTime = 75;
                    break;

                case TURNTORAMP:
                    moveTurn(-101.0, 0.6);
//                    moveTurnWithGyro(-101, 0.5);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.UPRAMP;
                    telemetryMove = MoveState.TURNTORAMP;
                    moveDelayTime = 75;
                    break;

                case UPRAMP:
                    moveStraight(-40.0, 0.3);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.DONE;
                    telemetryMove = MoveState.UPRAMP;
                    break;

                case DONE:
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    telemetryMove = MoveState.DONE;
                    break;
            }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Ultrasonic", ultraSense.getUltrasonicLevel());
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("ENCLeft", (float) leftDrive.getCurrentPosition());
        telemetry.addData("ENCRight", (float) rightDrive.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}

