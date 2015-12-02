package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
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
    double beaconPusherTarget;
    double climberDumperTarget;
    double beaconPinionTarget;


    //dc motor controllers
    DcMotorController driveTrainController;
    //dc motors
    DcMotor motorRight;
    DcMotor motorLeft;
    //servo controllers
    ServoController beaconServos;
    //servos
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    //sensors
    ColorSensor ColorSense;
    OpticalDistanceSensor OpticalDistance;
    GyroSensor gyroSense;
    UltrasonicSensor ultraSense;
    //Movestate options
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    double ifRedOnBeacon;
    int beaconPassDistance;
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

    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees
     * @param speed
     */
    void moveTurnWithGyro(int degrees, double speed) {
        int preTurnHeading;
        preTurnHeading = gyroSense.getHeading();
        if (gyroSense.getHeading() < preTurnHeading + degrees) {
            if (degrees > 0) {
                motorLeft.setPower(-speed);
                motorRight.setPower(speed);
                motorLeft.setTargetPosition(1000000000);
                motorRight.setTargetPosition(1000000000);
            }
            if (degrees < 0) {
                motorLeft.setPower(speed);
                motorRight.setPower(-speed);
            }
            motorLeft.setTargetPosition(1000000000);
            motorRight.setTargetPosition(1000000000);

        }
        if (gyroSense.getHeading() >= preTurnHeading + degrees){
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
        }
    }

    void motorOn(double speed, boolean onoroff) {
        if(onoroff = true){
            motorLeft.setPower(speed);
            motorRight.setPower(speed);
            motorLeft.setTargetPosition(1000000000);
            motorRight.setTargetPosition(1000000000);
        }
        if (onoroff = false) {
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
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
        beaconPusherTarget = beaconPresserPosition;
    }

    /**
     * since climberDumperPosition is clipped, values can be entered between 0.0 and 0.1
     *
     * @param climberDumperPosition
     */
    void moveClimberDump(double climberDumperPosition) {
        climberDumperPosition = Range.clip(climberDumperPosition, cdumper_MIN_RANGE, cdumber_MAX_RANGE);
        servoClimberDumper.setPosition(climberDumperPosition);
        climberDumperTarget = climberDumperPosition;
    }



    @Override
    public void init() {
        //dc Motor Controllers
        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
        //dc Motors
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        //servo controllers
        beaconServos = hardwareMap.servoController.get("beaconCtlr");
        //servos
        servoBeaconPinion = hardwareMap.servo.get("beacon_pinion");
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        //sensors
        ColorSense = hardwareMap.colorSensor.get("color");
        ColorSense.enableLed(true);
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        //motor configurations
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //movestate settings
        currentMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        lookingForRedFlag = false;
        sawBlueFlag = false;
        //servo positions
        moveBeaconPinion(0.0);
        moveClimberDump(0.0);
        moveBeaconPress(0.0);

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
//                    if (lookingForRedFlag && (ColorSense.red() >= 1))  {
//                        motorRight.setPower(0.0);
//                        motorLeft.setPower(0.0);
//                        currentMove = MoveState.MOVEDELAY;
//                    }
                    if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                        currentMove = MoveState.MOVEDELAY;
                    }
                    break;

                case SERVODUMPERMOVE:
                    if (servoClimberDumper.getPosition() > climberDumperTarget ) {
                        currentMove = MoveState.SERVODUMPERMOVE;
                    } else if (servoClimberDumper.getPosition() < climberDumperTarget) {
                        currentMove = MoveState.SERVODUMPERMOVE;
                    } else {
                        currentMove = MoveState.MOVEDELAY;
                    }
                    break;

                case SERVOPUSHERMOVE:
                    if (servoBeaconPusher.getPosition() > climberDumperTarget) {
                        currentMove = MoveState.SERVOPUSHERMOVE;
                    } else if (servoBeaconPusher.getPosition() < climberDumperTarget){
                        currentMove = MoveState.SERVOPUSHERMOVE;
                    } else {
                        currentMove = MoveState.MOVEDELAY;
                    }
                    break;

                case SERVOPINIONNULLMOVE:
                    if (ColorSense.red() < 1.0 && ColorSense.blue() < 1.0) {
                        moveBeaconPinion(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    } else if (ColorSense.red() >= 1.0 || ColorSense.blue() >= 1.0) {
                        moveBeaconPinion(beaconPinionTarget);
                        currentMove = MoveState.SERVOPINIONNULLMOVE;
                    }
                    break;

                case SERVOPINIONREDMOVE:
                    if (ColorSense.red() >= 1.0) {
                        moveBeaconPinion(0.0);
                        currentMove = MoveState.MOVEDELAY;
                    } else if (ColorSense.red() < 1.0) {
                        moveBeaconPinion(beaconPinionTarget);
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
                    moveStraight(219.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.FINDWALL;
                    telemetryMove = MoveState.MOVEDIAG;
                    moveDelayTime = 250;
                    break;

                case FINDWALL:
                    distanceToWall = ultraSense.getUltrasonicLevel();
                    if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                        moveStraight((distanceToWall - 15.0) * 1.414, 0.5);
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
                    moveStraight(-60.0, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.FINDBEACON;
                    telemetryMove = MoveState.DRIVEALONGWALL;
                    moveDelayTime = 75;
                    break;

                case FINDBEACON:
                    motorOn(0.25, true);
//                    lookingForRedFlag = true;
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.ALIGNDUMPER;
                    telemetryMove = MoveState.FINDBEACON;
                    moveDelayTime = 75;
                    break;

                case ALIGNDUMPER:
                    ifRedOnBeacon = ColorSense.red();
                    moveBeaconPinion(0.5);
                    currentMove = MoveState.SERVOPINIONNULLMOVE;
                    nextMove = MoveState.DUMPCLIMBERS;
                    telemetryMove = MoveState.ALIGNDUMPER;
                    moveDelayTime = 75;
                    break;

                case DUMPCLIMBERS:
                    moveClimberDump(1.0);
                    currentMove = MoveState.SERVODUMPERMOVE;
                    nextMove = MoveState.ALIGNPRESSER;
                    telemetryMove = MoveState.DUMPCLIMBERS;
                    moveDelayTime = 75;
                    break;

                case ALIGNPRESSER:
                    if (ifRedOnBeacon >= 1.0) {
                        moveBeaconPinion(-0.5);
                    }else if (ifRedOnBeacon < 1.0){
                        moveBeaconPinion(0.5);
                    }
                    currentMove = MoveState.SERVOPINIONREDMOVE;
                    nextMove = MoveState.PRESSBUTTON;
                    telemetryMove = MoveState.ALIGNPRESSER;
                    moveDelayTime = 75;
                    break;

                case PRESSBUTTON:
                    moveBeaconPress(1.0);
                    currentMove = MoveState.SERVOPUSHERMOVE;
                    nextMove = MoveState.PULLAHEADALONGWALL;
                    telemetryMove = MoveState.PRESSBUTTON;
                    moveDelayTime = 75;
                    break;

                case PULLAHEADALONGWALL:
                    moveStraight(30, 0.6);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.ROTATEFROMBEACON;
                    telemetryMove = MoveState.PULLAHEADALONGWALL;
                    moveDelayTime = 75;
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
                    motorLeft.setPower(0.0);
                    motorRight.setPower(0.0);
                    telemetryMove = MoveState.DONE;
                    break;
            }


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
        telemetry.addData("Current Move", telemetryMove.toString());
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}
