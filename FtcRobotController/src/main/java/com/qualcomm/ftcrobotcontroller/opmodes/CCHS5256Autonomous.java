// CCHS 5256 Autonomous Software
// Run in Autonomous Mode of FTC Challenge 2015-2016
// Autonomous for FIRST ResQ challenge;'

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 1/19/2016.
 * Covenant Christian High School
 * 5256 SpareParts Autonomous
 * FIRST ResQ
 */
public class CCHS5256Autonomous extends OpMode {

    enum MoveState {
        STARTMOVE, MOVINGSTRAIGHT, STARTTURN, MOVINGTURN, DELAYSETTINGS, DELAY,
        INITIALIZEROBOT, CHOOSEMOVE, FIRSTMOVE, TURNDIAG, MOVEDIAG, TURNONCOLOREDLINE, GOTOWHITELINE,
        TURNTOBEACON, DRIVETOBEACON, READBEACON, PUSHBUTTON, UNPUSHBUTTON, BACKUP,
        EXTENDARM, DUMPCLIMBERS, JIGGLEFORWARD, JIGGLEBACKWARD, PULLARMIN, BACKUPFARTHER, TURNALONGLINE, DRIVEALONGLINE, TURNTOMOUNTAIN,
        DRIVETOMOUNTAIN, GOUPMOUNTAIN, PREPTELEOP, DONE
    }

    enum OmniCtlr {
        NOTMOVING, EXTENDING, IN, OUT, DELAYSETTINGSOMNI, DELAYOMNI
    }

    // DC Motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor chinUp;
    DcMotor endGameLights;
    // Servos
    Servo armLock;
    Servo climberDumper;
    Servo ultraSenseServo;
    Servo leftOmniPinion;
    Servo rightOmniPinion;
    Servo leftPlow;
    Servo rightPlow;
    Servo leftTrigger;
    Servo rightTrigger;
    // Sensors
    GyroSensor gyroSense;
    ColorSensor fColorSense;
    UltrasonicSensor ultraSense;
    // Switches
    DigitalChannel nearMtnSwitch;
    DigitalChannel redBlueBeaconSwitch;
//    DigitalChannel delayPotSwitch;
//    DigitalChannel thirdTileSwitch;
//    AnalogInput delayPotentiometer;
    double diagMtnDist;
    double turnMtnDist;
    double toMtnDist;
    double redBlue;
    double firstMoveDist;
    double turnDiagDegrees;
    double moveDiagDist;
    double delay;
    boolean nearMountain;
    boolean farMountain;
    boolean redAlliance;
    boolean blueAlliance;
    boolean thirdTile;
    boolean fourthTile;
    boolean delayRobot;
    boolean lookingWithUltraSense;
    double targetReading;
    // State Machine Settings
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    OmniCtlr currentOmni;
    OmniCtlr nextOmni;
    OmniCtlr chosenOmni;
    long delayUntil;
    long moveDelayTime;
    long commonDelayTime;
    long now;
    long nowOmni;
    long moveDelaytimeOmni;
    long delayUntilOmni;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    double turnSpeed;
    boolean lookingForRedFlag;
    boolean lookingForBlueFlag;
    boolean lookingForWhiteLine;
    // Elapsed Time
    ElapsedTime currentTime;
    // Method Variables
    // centimetersToCounts
    double countsPerMeter = 5076.0;
    // degreesToCounts
    double countsPerDonut = 6083.0;
    // moveStraight
    double speed;
    boolean movingForward;
    // moveTurn
    int gyroError;
    int desiredHeading;

    // Methods that are called in the loop

    /**
     *
     * @param centimeters is the input centimeters number
     * @return returns a counts number for a certain number of centimeters
     */

    int centimetersToCounts(double centimeters) {
        return (int)(centimeters * (countsPerMeter / 100.0));

        //double wheelDiameteliftCheckr = 10.1;    // wheel diameter in cm
        //double encoderCounts = 1120.0;  // encoder counts per revolution of the drive train motors
        // return (int) ((centimeters / (wheelDiameter * Math.PI)) * encoderCounts);
        // ^ Previous code to find the amount of counts for every wheel rotation
    }

    /**
     *
     * @param degrees is the input degrees number
     * @return returns a counts number for a certain number of degrees
     */

    int degreesToCounts(double degrees) {
        return (int)(degrees * (countsPerDonut / 360.0));

        //double wheelBase = 40.3225; // wheelbase of the primary drive wheels
        //double oneDegree = ((wheelBase * Math.PI) / 360);   // calculates the distance of one degree
        //return centimetersToCounts(oneDegree * degrees);
        // ^ Previous code to find the degrees per count using the diameter of the wheels
    }

    /**
     *
     * @param distanceCM is how far forward we want to go
     * @param robotSpeed is the speed we want the robot to go
     */

    void moveStraight(double distanceCM, double robotSpeed) {
        int rightTarget;
        int leftTarget;

        speed = robotSpeed;
        if (distanceCM > 0.0) {
            movingForward = true;
        } else {
            movingForward = false;
        }
        leftTarget = leftDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() + centimetersToCounts(distanceCM);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(robotSpeed);
        rightDrive.setPower(robotSpeed);
    }


    /**
     * if degree magnitude is negative, robot turns clockwise
     *
     * @param degrees is number of degrees we want to turn
     *                negative is clockwise
     *                positive is counterclockwise
     * @param robotSpeed is the speed we want the wheels
     */
    void moveTurn(double degrees, double robotSpeed) {
        int rightTarget;
        int leftTarget;

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
        speed = robotSpeed;
        leftTarget = leftDrive.getCurrentPosition() + degreesToCounts(degrees + gyroError);
        leftDrive.setTargetPosition(leftTarget);
        rightTarget = rightDrive.getCurrentPosition() - degreesToCounts(degrees + gyroError);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(robotSpeed);
        rightDrive.setPower(robotSpeed);
    }

    @Override
    public void init() {
        // DC Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        chinUp = hardwareMap.dcMotor.get("chinUp");
        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        // DC Motor Settings
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        chinUp.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        chinUp.setPower(0.0);
        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        endGameLights.setPower(1.0);
        // Servos
        armLock = hardwareMap.servo.get("armLock");
        climberDumper = hardwareMap.servo.get("climber_dumper");
        ultraSenseServo = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        leftPlow = hardwareMap.servo.get("lP");
        rightPlow = hardwareMap.servo.get("rP");
        leftTrigger = hardwareMap.servo.get("lT");
        rightTrigger = hardwareMap.servo.get("rT");
        // Servo Settings
        armLock.setPosition(0.25);
        climberDumper.setPosition(0.5);
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        leftPlow.setPosition(0.5);
        rightPlow.setPosition(0.5);
        leftTrigger.setPosition(0.8);
        rightTrigger.setPosition(0.1);
        // Sensors
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        fColorSense = hardwareMap.colorSensor.get("fCS");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        // Switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
//        delayPotSwitch = hardwareMap.digitalChannel.get("dPotSw");
//        thirdTileSwitch = hardwareMap.digitalChannel.get("tileSw");
//        delayPotentiometer = hardwareMap.analogInput.get("delPot");
        //statemachine settings
        currentMove = MoveState.INITIALIZEROBOT;
        nextMove = MoveState.FIRSTMOVE;
        telemetryMove = MoveState.FIRSTMOVE;
        currentOmni = OmniCtlr.NOTMOVING;
        chosenOmni = OmniCtlr.NOTMOVING;
        // Set Switch Flags
        if (redBlueBeaconSwitch.getState()) {   // WE ARE RED
            redBlue = 1.0;
            ultraSenseServo.setPosition(0.25);
            redAlliance = true;
            blueAlliance = false;
            turnDiagDegrees = 45.0;
        } else {                                // WE ARE BLUE
            redBlue = -1.0;
            ultraSenseServo.setPosition(0.75);
            redAlliance = false;
            blueAlliance = true;
            turnDiagDegrees = -45.0;
        }

        if (nearMtnSwitch.getState()) {         // WE GO TO NEAR MOUNTAIN
            diagMtnDist = 0.0;
            turnMtnDist = -90.0;
            toMtnDist = 100.0;
            nearMountain = true;
            farMountain = false;
        } else {                                // WE GO TO FAR MOUNTAIN
            diagMtnDist = 50.0;
            turnMtnDist = 90.0;
            toMtnDist = 200.0;
            nearMountain = false;
            farMountain = true;
        }
//
//        if (delayPotSwitch.getState()) {        // WE DELAY
//            delay = (delayPotentiometer.getValue() * (10000 / 1024));
//            delayRobot = true;
//        } else {                                // WE DON'T DELAY
//            delay = 0.0;
//            delayRobot = false;
//        }
        delay = 0.0;
            delayRobot = false;
//
//        if (thirdTileSwitch.getState()) {       // WE ARE ON THE THIRD TILE FROM THE MOUNTAIN CORNER
//            firstMoveDist = 30.38;
//            turnDiagDegrees = 45.0;
//            moveDiagDist = 0.0;
//            thirdTile = true;
//            fourthTile = false;
//        } else {                                // WE ARE ON THE FOURTH TILE FROM THE MOUNTAIN CORNER
//            firstMoveDist = 0.0;
//            turnDiagDegrees = 0.0;
//            moveDiagDist = 30.38;
//            thirdTile = false;
//            fourthTile = true;
//        }
            firstMoveDist = 81.44;
            moveDiagDist = 0.0;
            thirdTile = true;
            fourthTile = false;

        // State Machine Settings
        fastSpeed = 0.8;
        mediumSpeed = 0.5;
        slowSpeed = 0.2;
        turnSpeed = 0.4;
        commonDelayTime = 200;
        lookingForWhiteLine = false;
        // Elapsed Time
        currentTime = new ElapsedTime();
        // log switch positions
//        Log.i("delay", Double.toString(delay));
        // Calibrate Gyro
        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
    }

    @Override
    public void loop() {
        if (gyroSense.isCalibrating()) {
            return;
        }
        double distanceToWall = 0.0;
        endGameLights.setPower(1.0);

        switch (currentMove) {
            //  WE USE THESE IN ALL MOVES
            case STARTMOVE:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.MOVINGSTRAIGHT;
                }
                break;

            case MOVINGSTRAIGHT:
//                if (lookingForWhiteLine) {
//                    if (fColorSense.alpha() >= 1.0) {
//                        leftDrive.setPower(0.0);
//                        rightDrive.setPower(0.0);
//                        currentMove = MoveState.DELAYSETTINGS;
//                    }
//                }
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
                rightDrive.setPower(Range.clip(speed - (gyroError * 0.01), -1.0, 1.0));
                leftDrive.setPower(Range.clip(speed + (gyroError * 0.01), -1.0, 1.0));
//                if (colorSense.blue() >= 1) {
//                    sawBlueFlag = true;
//                }
//                if (colorSense.red() >= 1) {
//                    sawRedFlag = true;
//                }
//                if (lookingForRedFlag && (colorSense.red() >= 1))  {
//                    rightDrive.setPower(0.0);
//                    leftDrive.setPower(0.0);
//                    currentMove = MoveState.DELAYSETTINGS;
//                }
//                if (lookingForBlueFlag && (colorSense.blue() >= 1))  {
//                    rightDrive.setPower(0.0);
//                    leftDrive.setPower(0.0);
//                    currentMove = MoveState.DELAYSETTINGS;
//                }
                if (lookingWithUltraSense) {
                    distanceToWall = ultraSense.getUltrasonicLevel();
                    if (distanceToWall > 0 && distanceToWall < 255) {
                        if (distanceToWall <= targetReading) {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            currentMove = MoveState.DELAYSETTINGS;
                        }
                    }
                }
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.DELAYSETTINGS;
                }
                break;

            case STARTTURN:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.MOVINGTURN;
                }
                break;

            case MOVINGTURN:
                gyroError = gyroSense.getHeading() - desiredHeading;
                if (gyroError > 180) {
                    gyroError = 360 - gyroError;
                }
                if (gyroError < -180) {
                    gyroError = 360 + gyroError;
                }
                //                moveTurn(gyroError, speed);
                if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                    currentMove = MoveState.DELAYSETTINGS;
                }
                break;

            case DELAYSETTINGS:
                now = System.currentTimeMillis();
                delayUntil = now + moveDelayTime;
                currentMove = MoveState.DELAY;
                Log.i("MOVEDELAY: time", Long.toString(System.currentTimeMillis()));
                Log.i("MOVEDELAY: now", Long.toString(now));
                Log.i("MOVEDELAY: delayTime", Long.toString(moveDelayTime));
                Log.i("MOVEDELAY: delayUntil", Long.toString(delayUntil));
                break;

            case DELAY:
                if (System.currentTimeMillis() >= delayUntil) {
                    currentMove = nextMove;
                }
                break;

            // MOVES WE USE ONCE, IN A SEQUENCE
            case INITIALIZEROBOT:
                currentTime.reset();
                moveDelayTime = (long)delay;
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.FIRSTMOVE;
                telemetryMove = MoveState.INITIALIZEROBOT;
                moveDelayTime = commonDelayTime;
                break;

//            case CHOOSEMOVE:
//                if (thirdTile) {
//                    currentMove = MoveState.FIRSTMOVE;
//                } else if (fourthTile){
//                    currentMove = MoveState.MOVEDIAG;
//                }
//                telemetryMove = MoveState.CHOOSEMOVE;
//                moveDelayTime = commonDelayTime;
//                break;

            case FIRSTMOVE:
                // Move Straight firstMoveDist
                moveStraight(firstMoveDist, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.FIRSTMOVE;
                moveDelayTime = commonDelayTime;
                break;

            case TURNDIAG:
                // Move Turn turnDiagDegrees
                moveTurn(turnDiagDegrees, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.MOVEDIAG;
                telemetryMove = MoveState.TURNDIAG;
                moveDelayTime = commonDelayTime;
                break;

            case MOVEDIAG:
                // Move straight 70 + moveDiagDist to red
//                lookingForWhiteLine = true;
                moveStraight(167 + moveDiagDist, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNONCOLOREDLINE;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = commonDelayTime;
                break;

            case TURNONCOLOREDLINE:
                // Move Turn -45 degrees
                moveTurn(-45.0, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DRIVETOBEACON;
                telemetryMove = MoveState.TURNONCOLOREDLINE;
                moveDelayTime = commonDelayTime;
                break;

//            case GOTOWHITELINE:
//                 Move Straight to white line
//                lookingForWhiteLine = true;
//                moveStraight(45.0, slowSpeed);
//                currentMove = MoveState.STARTMOVE;
//                nextMove = MoveState.DONE;
//                telemetryMove = MoveState.GOTOWHITELINE;
//                moveDelayTime = commonDelayTime;
//                break;

//            case TURNTOBEACON:
//                 Move Turn 90 degrees to the beacon
//                moveTurn(90.0, turnSpeed);
//                currentMove = MoveState.STARTTURN;
//                nextMove = MoveState.DRIVETOBEACON;
//                telemetryMove = MoveState.TURNTOBEACON;
//                moveDelayTime = commonDelayTime;
//                break;

            case DRIVETOBEACON:
                targetReading = 20.0;
                lookingWithUltraSense = true;
                moveStraight(45.0, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.EXTENDARM;
                telemetryMove = MoveState.DRIVETOBEACON;
                moveDelayTime = commonDelayTime;
                break;

//            case PUSHBUTTON:
//                 push the button
//                lookingWithUltraSense = false;
//                nextMove = MoveState.UNPUSHBUTTON;
//                telemetryMove = MoveState.PUSHBUTTON;
//                break;
//
//            case UNPUSHBUTTON:
//                 unpush the button
//                nextMove = MoveState.BACKUP;
//                telemetryMove = MoveState.UNPUSHBUTTON;
//                break;

//            case BACKUP:
//                 Move Straight till can dump climbers
//                lookingWithUltraSense = false;
//                moveStraight(-5.0, mediumSpeed);
//                currentMove = MoveState.STARTMOVE;
//                nextMove = MoveState.EXTENDARM;
//                telemetryMove = MoveState.BACKUP;
//                moveDelayTime = commonDelayTime;
//                break;

            case EXTENDARM:
                // Dump climbers
                chinUp.setTargetPosition(-3360);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.DUMPCLIMBERS;
                telemetryMove = MoveState.EXTENDARM;
                moveDelayTime = 500;
                break;

            case DUMPCLIMBERS:
               if (!chinUp.isBusy()) {
                   climberDumper.setPosition(1.0);
                   currentMove = MoveState.DELAYSETTINGS;
                   nextMove = MoveState.JIGGLEFORWARD;
                   telemetryMove = MoveState.DUMPCLIMBERS;
                   moveDelayTime = commonDelayTime;
               }
                break;

            case JIGGLEFORWARD:
                moveStraight(3.0, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.JIGGLEBACKWARD;
                telemetryMove = MoveState.JIGGLEFORWARD;
                moveDelayTime = commonDelayTime;
                break;

            case JIGGLEBACKWARD:
                moveStraight(-3.0, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.PULLARMIN;
                telemetryMove = MoveState.JIGGLEBACKWARD;
                moveDelayTime = commonDelayTime;
                break;

            case PULLARMIN:
                chinUp.setTargetPosition(3360);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.TURNALONGLINE;
                telemetryMove = MoveState.EXTENDARM;
                moveDelayTime = 700;
                break;


            case TURNALONGLINE:
                // Turn so we can position ourselves to go up the mountain
                moveTurn(90.0, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.BACKUPFARTHER;
                telemetryMove = MoveState.TURNALONGLINE;
                moveDelayTime = commonDelayTime;
                break;

            case  BACKUPFARTHER:
                // Move Straight 15 so we can drive to mountain
                moveStraight(-60.0, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.BACKUPFARTHER;
                chosenOmni = OmniCtlr.EXTENDING;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVEALONGLINE:
                // Move Straight 25 + diagMtnDist
                moveStraight(25 + diagMtnDist, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTOMOUNTAIN;
                telemetryMove = MoveState.DRIVEALONGLINE;
                moveDelayTime = commonDelayTime;
                break;

            case TURNTOMOUNTAIN:
                // Move Turn turnMtnDegrees
                moveTurn(turnMtnDist, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DRIVETOMOUNTAIN;
                telemetryMove = MoveState.TURNTOMOUNTAIN;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVETOMOUNTAIN:
                // Move Straight toMtnDist
                moveStraight(toMtnDist, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.GOUPMOUNTAIN;
                telemetryMove = MoveState.DRIVETOMOUNTAIN;
                moveDelayTime = commonDelayTime;
                break;

            case GOUPMOUNTAIN:
                // Move Straight up mountain
                moveStraight(100.0, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.PREPTELEOP;
                telemetryMove = MoveState.GOUPMOUNTAIN;
                moveDelayTime = commonDelayTime;
                break;

            case PREPTELEOP:
                // get robot ready for TeleOp
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.PREPTELEOP;
                moveDelayTime = commonDelayTime;
                break;

            case DONE:
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                endGameLights.setPower(0.5);
                break;
        }

        switch (currentOmni) {
            case DELAYSETTINGSOMNI:
                nowOmni = System.currentTimeMillis();
                delayUntilOmni = nowOmni + moveDelaytimeOmni;
                currentOmni = OmniCtlr.DELAYOMNI;
                break;

            case DELAYOMNI:
                if (System.currentTimeMillis() >= delayUntilOmni) {
                    currentOmni = nextOmni;
                }
                break;

            case NOTMOVING:
                leftOmniPinion.setPosition(0.5);
                rightOmniPinion.setPosition(0.5);
                currentOmni = chosenOmni;
                break;

            case EXTENDING:
                leftOmniPinion.setPosition(1.0);
                rightOmniPinion.setPosition(1.0);
                currentOmni = OmniCtlr.DELAYSETTINGSOMNI;
                nextOmni = OmniCtlr.OUT;
                moveDelaytimeOmni = 5500;
                break;

            case OUT:
                leftOmniPinion.setPosition(0.5);
                rightOmniPinion.setPosition(0.5);
                currentOmni = OmniCtlr.NOTMOVING;
                break;

        }

        telemetry.addData("left encoder", leftDrive.getCurrentPosition());
        telemetry.addData("right encoder", rightDrive.getCurrentPosition());
        telemetry.addData("current move", telemetryMove.toString());
        telemetry.addData("Elapsed Time", currentTime.time());
        telemetry.addData("ColorSensor", Integer.toString(fColorSense.alpha()));
        telemetry.addData("red", Integer.toString(fColorSense.red()));
        telemetry.addData("blue", Integer.toString(fColorSense.blue()));
        telemetry.addData("green", Integer.toString(fColorSense.green()));
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());


    }

    @Override
    public void stop () {
    }

}
