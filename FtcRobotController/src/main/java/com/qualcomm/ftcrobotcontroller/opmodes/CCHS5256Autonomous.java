package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Date;

/**
 * Created by Robotics on 10/7/2015.
 */
public class CCHS5256Autonomous extends OpMode {
    enum MoveState {
        DELAY, STARTMOVE, MOVING, SERVO_BEACON_PUSHER_pMOVEMENT, SERVO_BEACON_PINION_pMOVEMENT, SERVO_CLIMBER_DUMPER_pMOVEMENT,
        SERVO_CLIMBER_RELEASER_pMOVEMENT, SERVO_BEACON_PUSHER_nMOVEMENT, SERVO_BEACON_PINION_nMOVEMENT, SERVO_CLIMBER_DUMPER_nMOVEMENT,
        SERVO_CLIMBER_RELEASER_nMOVEMENT,START1, START2, START3, START4, START5, START6, START7, START8, DELAY1, DELAY2, DELAY3,
        DELAY4, DELAY5, DELAY6, DELAY7, DELAY8, DELAY9, DELAY10, DELAY11, DELAY12, ALIGN_PRESSER, PRESS_BUTTON, TIP_CLIMBERS, RELEASE_CLIMBERS, ALIGN_DUMPER, DRIVE_TO_POSITION,DONE

    }

    enum MoveState {
        DELAY, STARTMOVE, MOVING, MOVEDELAY, FIRSTMOVE, TURNDIAG, MOVEDIAG, FINDWALL, TURNALONGWALL,
        FINDBEACON, ROTATEFROMBEACON, MOVETORAMP, TURNTORAMP, DONE
    }


//    enum BeaconState {
//        NOT_LOOKING, LOOKING_START, LOOKING_END, DONE
//    }



//    enum BeaconColor {
//        RED, BLUE
//    }
//
//    enum RobotSide {
//        LEFT, RIGHT
//    }

    final static double bpinion_MIN_RANGE  = 0.20;
    //not sure, we are using a continuous
    final static double bpinnion_MAX_RANGE  = 0.90;
    //not sure, we are using a continuous
    final static double bpusher_MIN_RANGE  = 0.20;
    final static double bpusher_MAX_RANGE  = 0.80;
    final static double cdumper_MIN_RANGE  = 0.00;
    final static double cdumber_MAX_RANGE  = 1.00;
    final static double creleaser_MIN_RANGE  = 0.00;
    final static double creleaser_MAX_RANGE  = 1.00;

    // position of the arm servo.
    double beaconPinionPosition;

    // amount to change the arm servo position.
    double beaconPinionDelta = 0.1;

    // position of the claw servo
    double beaconPusherPosition;

    // amount to change the claw servo position by
    double beaconPusherDelta = 0.1;

    // position of the arm servo.
    double climberDumperPosition;

    // amount to change the arm servo position.
    double climberDumperDelta = 0.1;

    // position of the claw servo
    double climberReleaserPosition;

    // amount to change the claw servo position by
    double climberReleaserDelta = 0.1;

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    ColorSensor ColorSense;
    //GyroSensor Gyro;
    //IrSeekerSensor IrSense;
    OpticalDistanceSensor OpticalDistance;
    Servo servoBeaconPinion;
    Servo servoBeaconPusher;
    Servo servoClimberDumper;
    Servo servoClimberReleaser;
    MoveState currentMove;
    MoveState nextMove;
    double ifRedOnBeacon;
//    BeaconState redState;
//    //double[][][] beaconLocation;
//    int redStartRight;
//    int redStartLeft;
//    int redEndRight;
//    int redEndLeft;
//    BeaconState blueState;
//    int blueStartRight;
//    int blueStartLeft;
//    int blueEndRight;
//    int blueEndLeft;
    long delayUntil;
    int beaconPassDistance;
//    boolean lookingForRedFlag;
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

    void motorOn(double speed, boolean onoroff) {
        if(onoroff = true){
            motorLeft.setPower(speed);
            motorRight.setPower(speed);
        }
        if (onoroff = false) {
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
        }


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
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        servoBeaconPinion = hardwareMap.servo.get("beacon_pinion");
        servoBeaconPusher = hardwareMap.servo.get("beacon_pusher");
        servoClimberDumper = hardwareMap.servo.get("climber_dumper");
        servoClimberReleaser = hardwareMap.servo.get("climber_releaser");
        servoBeaconPinion.setPosition(beaconPinionPosition);
        servoBeaconPusher.setPosition(beaconPusherPosition);
        servoClimberReleaser.setPosition(climberReleaserPosition);
        servoClimberDumper.setPosition(climberDumperPosition);
        currentMove = MoveState.START1;
//        redState = BeaconState.NOT_LOOKING;
//        blueState = BeaconState.NOT_LOOKING;

        beaconPusherPosition = 0.20;
        climberDumperPosition = 0.00;
        climberReleaserPosition = 0.00;

        //beaconLocation = new double[2][2][2];
    }

    @Override
    public void loop() {
        servoBeaconPinion.setPosition(beaconPinionPosition);
        servoBeaconPusher.setPosition(beaconPusherPosition);
        servoClimberReleaser.setPosition(climberReleaserPosition);
        servoClimberDumper.setPosition(climberDumperPosition);

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
                    moveStraight(219.0, 0.5);
                    currentMove = MoveState.STARTMOVE;
                    nextMove = MoveState.FINDWALL;
                    moveDelayTime = 1000;
                    break;

                case FINDWALL:
                    distanceToWall = ultraSense.getUltrasonicLevel();
                    if ((distanceToWall > 30.0) && (distanceToWall <= 70.0)) {
                        moveStraight((distanceToWall - 15.0) * 1.414, 0.5);
                        currentMove = MoveState.STARTMOVE;
                        nextMove = MoveState.TURNALONGWALL;
                        moveDelayTime = 1000;
                    }
                    break;

                case TURNALONGWALL:
                    moveTurn(-45.0, 0.5);
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

        switch (currentMove) {
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
                if (ColorSense.red() >= 1) {
                    motorRight.setPower(0.0);
                    motorLeft.setPower(0.0);
                    currentMove = nextMove;
                }
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_BEACON_PINION_pMOVEMENT:
                break;

            case SERVO_BEACON_PUSHER_pMOVEMENT:
                if (servoBeaconPusher.getPosition() < beaconPusherPosition) {
                    currentMove = MoveState.SERVO_BEACON_PUSHER_pMOVEMENT;
                }
                if (servoBeaconPusher.getPosition() >= beaconPusherPosition) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_CLIMBER_DUMPER_pMOVEMENT:
                if (servoClimberDumper.getPosition() < climberDumperPosition) {
                    currentMove = MoveState.SERVO_CLIMBER_DUMPER_pMOVEMENT;
                }
                if (servoClimberDumper.getPosition() >= climberDumperPosition) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_CLIMBER_RELEASER_pMOVEMENT:
                if (servoClimberReleaser.getPosition() < climberReleaserPosition) {
                    currentMove = MoveState.SERVO_CLIMBER_RELEASER_pMOVEMENT;
                }
                if (servoClimberReleaser.getPosition() >= climberReleaserPosition) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_BEACON_PINION_nMOVEMENT:
                break;

            case SERVO_BEACON_PUSHER_nMOVEMENT:
                if (servoBeaconPusher.getPosition() > beaconPusherPosition) {
                    currentMove = MoveState.SERVO_BEACON_PUSHER_nMOVEMENT;
                }
                if (servoBeaconPusher.getPosition() <= beaconPusherPosition) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_CLIMBER_DUMPER_nMOVEMENT:
                if (servoClimberDumper.getPosition() > climberDumperPosition) {
                    currentMove = MoveState.SERVO_CLIMBER_DUMPER_nMOVEMENT;
                }
                if (servoClimberDumper.getPosition() <= climberDumperPosition) {
                    currentMove = nextMove;
                }
                break;

            case SERVO_CLIMBER_RELEASER_nMOVEMENT:
                if (servoClimberReleaser.getPosition() > climberReleaserPosition) {
                    currentMove = MoveState.SERVO_CLIMBER_RELEASER_nMOVEMENT;
                }
                if (servoClimberReleaser.getPosition() <= climberReleaserPosition) {
                    currentMove = nextMove;
                }
                break;
            case START1:
                moveStraight(85.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY1;
                break;

            case DELAY1:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START2;
                break;

            case START2:
                moveTurn(45.0, 0.5);
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
                moveStraight(266.7, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY3;
                break;

            case DELAY3:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START4;
                break;

            case START4:
                moveTurn(-45.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY4;
                break;

            case DELAY4:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START5;
                break;

            case START5:
                if (ColorSense.red() >= 1.0 || ColorSense.blue() >= 1.0) {
                    motorOn(0.0,false);
                }
                if (ColorSense.red() < 1.0 || ColorSense.blue() < 1.0) {
                    motorOn(0.5, true);
                }
                //lookingForRedFlag = true;
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY5;
                break;

            case DELAY5:
                //lookingForRedFlag = false;
                now = new Date();
                delayUntil = now.getTime() + 1000;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.ALIGN_DUMPER;
                break;

            case ALIGN_DUMPER:
                ifRedOnBeacon = ColorSense.red();
                servoBeaconPinion.setDirection(Servo.Direction.FORWARD);
                servoBeaconPinion.setPosition(1.0);
                nextMove = MoveState.DELAY6;
                break;

            case DELAY6:
                now = new Date();
                delayUntil = now.getTime() + 1000;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.TIP_CLIMBERS;
                break;

            case TIP_CLIMBERS:
                climberDumperPosition = 1.0;
                currentMove = MoveState.SERVO_CLIMBER_DUMPER_pMOVEMENT;
                nextMove = MoveState.DELAY7;
                break;

            case DELAY7:
                now = new Date();
                delayUntil = now.getTime() + 1000;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.RELEASE_CLIMBERS;
                break;

            case RELEASE_CLIMBERS:
                climberReleaserPosition = 1.0;
                currentMove = MoveState.SERVO_CLIMBER_RELEASER_pMOVEMENT;
                nextMove = MoveState.DELAY8;
                break;

            case DELAY8:
                now = new Date();
                delayUntil = now.getTime() + 1000;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.RELEASE_CLIMBERS;
                break;

            case ALIGN_PRESSER:
                if (ifRedOnBeacon >= 1.0) {
                    servoBeaconPinion.setDirection(Servo.Direction.REVERSE);
                }
                if (ifRedOnBeacon < 1.0) {
                    servoBeaconPinion.setDirection(Servo.Direction.FORWARD);
                }
                nextMove = MoveState.PRESS_BUTTON;

            case PRESS_BUTTON:
                beaconPusherPosition = 0.8;
                currentMove = MoveState.SERVO_BEACON_PUSHER_pMOVEMENT;
                nextMove = MoveState.DELAY9;
                break;

            case DELAY9:
                now = new Date();
                delayUntil = now.getTime() + 1000;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.DRIVE_TO_POSITION;
                break;

            case DRIVE_TO_POSITION:
                moveStraight(countsToCentimeters(motorRight.getCurrentPosition()-beaconPassDistance),0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY10;
                break;

            case DELAY10:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START6;
                break;

            case START6:
                moveTurn(50.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY11;
                break;

            case DELAY11:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START7;
                break;

            case START7:
                moveStraight(-103.0, 0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DELAY12;
                break;

            case DELAY12:
                now = new Date();
                delayUntil = now.getTime() + 100;
                currentMove = MoveState.DELAY;
                nextMove = MoveState.START8;
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
//        switch (redState) {
//            case NOT_LOOKING:
//                break;
//            case LOOKING_START:
//                if (ColorSense.red() >= 1) {
//                    redStartLeft = motorLeft.getCurrentPosition();
//                    redStartRight = motorRight.getCurrentPosition();
//                    redState = BeaconState.LOOKING_END;
//                }
//                break;
//            case LOOKING_END:
//                if (ColorSense.red() == 0) {
//                    redEndLeft = motorLeft.getCurrentPosition();
//                    redEndRight = motorRight.getCurrentPosition();
//                    redState = BeaconState.DONE;
//                }
//                break;
//            case DONE:
//                break;
//        }
//        switch (blueState) {
//            case NOT_LOOKING:
//                break;
//            case LOOKING_START:
//                if (ColorSense.blue() >= 1) {
//                    blueStartLeft = motorLeft.getCurrentPosition();
//                    blueStartRight = motorRight.getCurrentPosition();
//                    blueState = BeaconState.LOOKING_END;
//                }
//                break;
//            case LOOKING_END:
//                if (ColorSense.blue() == 0) {
//                    blueEndLeft = motorLeft.getCurrentPosition();
//                    blueEndRight = motorRight.getCurrentPosition();
//                    blueState = BeaconState.DONE;
//                }
//                break;
//            case DONE:
//                break;
//        }

        if (gamepad1.start) {
            currentMove = MoveState.START1;
        }



        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Look for Red");
//        telemetry.addData("RedStart", redStartLeft);
//        telemetry.addData("RedEnd", redEndLeft);
//        telemetry.addData("BlueStart", blueStartLeft);
//        telemetry.addData("BlueEnd", blueEndLeft);
//        telemetry.addData("GyroSense", (float) Gyro.getRotation());
        telemetry.addData("OpticalSense", (float) OpticalDistance.getLightDetectedRaw());
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
    }

    @Override
    public void stop() {
    }
}
