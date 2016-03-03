// CCHS 5256 Autonomous Software
// Run in Autonomous Mode of FTC Challenge 2015-2016
// Autonomous for FIRST ResQ challenge;
//Some lines of code are commented out,
//they either did not work and were not able to be fixed at the time,
//or they are too be added later when a specific part is put on the robot.

package com.qualcomm.ftcrobotcontroller.opmodes;
//all the classes we need to import to get our autonoumous running
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
 * Created by Robotics on 2/16/2016.
 */
public class BEAST_MODE_Autonomous extends OpMode {
    //movestate for our autonomous move
    enum MoveState {
        STARTMOVE, MOVINGSTRAIGHT, STARTTURN, MOVINGTURN, DELAYSETTINGS, DELAY, INITIALIZEROBOT,
        DRIVEAWAYFROMWALL, TURNDIAG, MOVEDIAG, GOPASTREDTAPE, TURNPARALLELTOWALL,
        DRIVETOWHITELINE, DRIVEPASTWHITELINE, TURNTOBEACON, DRIVETOBEACON, WINDOUTARM, DUMPCLIMBERS,
        JIGGLEF, JIGGLEB, WINDINARMPARTWAY, READBEACON,  TURNTOBUTTON, PUSHBUTTON, BACKUP,
        TURNTOFLOORGOAL, DRIVEINFLOORGOAL, DONE
    }
    //movestate for our omniwheels in autonomous
    enum OmniCtlr {
        PRENOTMOVING, NOTMOVING, EXTENDING, IN, OUT, DELAYSETTINGSOMNI, DELAYOMNI, RETRACT, REEXTEND, RERETRACT,
        DONE
    }
    //declaration of everything we will use
    // DC Motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor chinUp;
//    DcMotor endGameLights;
    DcMotor debrisSwivel;
    // Servos
    Servo armLock;
    Servo climberDumper;
    Servo leftOmniPinion;
    Servo rightOmniPinion;
    Servo leftPlow;
//    Servo rightPlow;
    Servo leftTrigger;
    Servo rightTrigger;
    Servo dumperDoor;
    Servo sweeper;
    // Sensors
    GyroSensor gyroSense;
    ColorSensor fColorSense;
    float hsvValues[] = {0F,0F,0F};
    ColorSensor bColorSense;
    UltrasonicSensor ultraSense;
    // Switches
//    DigitalChannel nearMtnSwitch;
    DigitalChannel redBlueBeaconSwitch;
//    DigitalChannel delayPotSwitch;
    DigitalChannel thirdTileSwitch;
//    AnalogInput delayPotentiometer;
    //variables
    double redBlue;
    double firstMoveDist;
    double moveDiagDist;
    double delay;
    boolean redAlliance;
    boolean blueAlliance;
    boolean thirdTile;
    boolean fourthTile;
    boolean lookingWithUltraSense;
    double targetReading;
    boolean redL;
    boolean redR;
    boolean blueL;
    boolean blueR;
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
    int counter;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    double turnSpeed;
    boolean lookingForWhiteLine;
    int beaconPushTurn;
    boolean lookingForRedTape;
    boolean lookingForBlueTape;
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
    //turns encoder counts into meters, then centimeters
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
    //turns encoder counts into a 360 degree turn
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
    //makes the robot move straight
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
    //allows the robot to turn
    void moveTurn(double degrees, double robotSpeed) {
        int rightTarget;
        int leftTarget;

        degrees = (degrees * redBlue);//negates turns if we are blue so that we turn the correct way

        // Figure out how far off we are at the end of the previous move so we can correct
        gyroError =  desiredHeading - gyroSense.getHeading();
        if(gyroError > 180) {
            gyroError = 360 - gyroError;
        }
        if (gyroError < -180) {
            gyroError = 360 + gyroError;
        }

        desiredHeading = desiredHeading + (int)(degrees);
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
        //initializes everything we are using so that it is ready to be used
        // DC Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        chinUp = hardwareMap.dcMotor.get("chinUp");
//        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        debrisSwivel = hardwareMap.dcMotor.get("blockDumper");
        // DC Motor Settings
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        chinUp.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        endGameLights.setPower(0.9);
        debrisSwivel.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        // Servos
        armLock = hardwareMap.servo.get("armLock");
        climberDumper = hardwareMap.servo.get("climber_dumper");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        leftPlow = hardwareMap.servo.get("lP");
//        rightPlow = hardwareMap.servo.get("rP");
        leftTrigger = hardwareMap.servo.get("lT");
        rightTrigger = hardwareMap.servo.get("rT");
        sweeper = hardwareMap.servo.get("sweeper");
        dumperDoor = hardwareMap.servo.get("dumperDoor");
        // Servo Settings
        armLock.setPosition(0.25);
        climberDumper.setPosition(0.15);
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        leftPlow.setPosition(0.5);
//        rightPlow.setPosition(0.68627450);
        leftTrigger.setPosition(0.8);
        rightTrigger.setPosition(0.1);
        sweeper.setPosition(0.5);
        dumperDoor.setPosition(0.0);
        // Sensors
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        bColorSense = hardwareMap.colorSensor.get("bCS");
        bColorSense.setI2cAddress(0x42);
        bColorSense.enableLed(false);
        fColorSense = hardwareMap.colorSensor.get("fCS");
//        fColorSense.setI2cAddress(0x40);
        fColorSense.enableLed(true);
        lookingForWhiteLine = false;
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        // Switches
//        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
//        delayPotSwitch = hardwareMap.digitalChannel.get("dPotSw");
        thirdTileSwitch = hardwareMap.digitalChannel.get("tileSw");
//        delayPotentiometer = hardwareMap.analogInput.get("delPot");
        //statemachine settings
        currentMove = MoveState.DRIVEAWAYFROMWALL;
//        telemetryMove = MoveState.INITIALIZEROBOT;
        currentOmni = OmniCtlr.NOTMOVING;
        chosenOmni = OmniCtlr.NOTMOVING;
        counter = 0;
        redL = false;
        redR = false;
        blueL = false;
        blueR = false;
        // Set Switch Flags
        if (redBlueBeaconSwitch.getState()) {   // WE ARE BLUE
            redBlue = -1.0;
            redAlliance = true;
            blueAlliance = false;
        } else {                                // WE ARE RED
            redBlue = 1.0;
            redAlliance = false;
            blueAlliance = true;
        }

//        if (nearMtnSwitch.getState()) {         // WE GO TO NEAR MOUNTAIN
//            diagMtnDist = 0.0;
//            turnMtnDist = -90.0;
//            toMtnDist = 100.0;
//            nearMountain = true;
//            farMountain = false;
//        } else {                                // WE GO TO FAR MOUNTAIN
//            diagMtnDist = 50.0;
//            turnMtnDist = 90.0;
//            toMtnDist = 200.0;
//            nearMountain = false;
//            farMountain = true;
//        }
//
//        if (delayPotSwitch.getState()) {        // WE DELAY
//            delay = (delayPotentiometer.getValue() * (10000 / 1024));
//            delayRobot = true;
//        } else {                                // WE DON'T DELAY
//            delay = 0.0;
//            delayRobot = false;
//        }
//        delay = 0.0;
//        delayRobot = false;

        if (thirdTileSwitch.getState()) {       // WE ARE ON THE THIRD TILE FROM THE MOUNTAIN CORNER
            firstMoveDist = 6.35;
            moveDiagDist = 165.0;
            thirdTile = true;
            fourthTile = false;
        } else {                                // WE ARE ON THE FOURTH TILE FROM THE MOUNTAIN CORNER
            firstMoveDist = 67.31;
            moveDiagDist = 122.0;
            thirdTile = false;
            fourthTile = true;
        }
        // State Machine Settings
        fastSpeed = 1.0;
        mediumSpeed = 0.7;
        slowSpeed = 0.3;
        turnSpeed = 0.5;
        commonDelayTime = 300;
        desiredHeading = 0;
        lookingForWhiteLine = false;
        lookingWithUltraSense = false;
        lookingForRedTape = false;
        lookingForBlueTape = false;
        beaconPushTurn = 0;
        // Elapsed Time
        currentTime = new ElapsedTime();
        // log switch positions
        Log.i("delay", Double.toString(delay));
        // Calibrate Gyro
        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
            telemetry.addData("gyro is calibrating", "");
        }
    }

    @Override
    public void loop() {
        //calibrates the gyro so that it is set at 0 degrees
        if (gyroSense.isCalibrating()) {
            return;
        }
        double distanceToWall = 0.0;
//        endGameLights.setPower(0.9);

        switch (currentMove) {
            //  WE USE THESE IN ALL MOVES
            case STARTMOVE:
                if (leftDrive.isBusy() && rightDrive.isBusy()) {
                    currentMove = MoveState.MOVINGSTRAIGHT;
                }
                break;

            case MOVINGSTRAIGHT:
                if (lookingForWhiteLine) {
                    if (hsvValues[0] >= 68.0) {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentMove = MoveState.DELAYSETTINGS;
                    }
                }

                if (lookingForRedTape) {
                    if (fColorSense.red() >= 30.0 && fColorSense.red() <= 100.0) {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentMove = MoveState.DELAYSETTINGS;
                    }
                }

                if (lookingForBlueTape) {
                    if (fColorSense.blue() >= 45.0 && fColorSense.blue()<= 100.0) {
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        currentMove = MoveState.DELAYSETTINGS;
                    }
                }
                //figures out the error and fixes it with the gyro
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
//                rightDrive.setPower(Range.clip(speed - (gyroError * 0.01), -1.0, 1.0));
//                leftDrive.setPower(Range.clip(speed + (gyroError * 0.01), -1.0, 1.0));
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
                climberDumper.setPosition(0.15);
                currentMove = MoveState.DRIVEAWAYFROMWALL;
                telemetryMove = MoveState.INITIALIZEROBOT;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVEAWAYFROMWALL:
                currentTime.reset();
                fColorSense.enableLed(true);
                moveStraight(firstMoveDist, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNDIAG;
                telemetryMove = MoveState.DRIVEAWAYFROMWALL;
                moveDelayTime = commonDelayTime;
                break;

            case TURNDIAG:
                moveTurn(45.0, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.MOVEDIAG;
                telemetryMove = MoveState.TURNDIAG;
                moveDelayTime = commonDelayTime;
                break;

            case MOVEDIAG:
                leftPlow.setPosition(0.6);
                if (redAlliance) {
                    lookingForRedTape = true;
                } else if (blueAlliance) {
                    lookingForBlueTape = true;
                }
                moveStraight(moveDiagDist + 20, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.GOPASTREDTAPE;
                telemetryMove = MoveState.MOVEDIAG;
                moveDelayTime = commonDelayTime;
                chosenOmni = OmniCtlr.EXTENDING;
                break;

            case GOPASTREDTAPE:
                if (redAlliance) {
                    lookingForRedTape = false;
                } else if (blueAlliance){
                    lookingForBlueTape = false;
                }
                moveStraight(5, mediumSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNPARALLELTOWALL;
                telemetryMove = MoveState.GOPASTREDTAPE;
                moveDelayTime = commonDelayTime;
                break;

            case TURNPARALLELTOWALL:
                moveTurn(-45.0, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DRIVETOWHITELINE;
                telemetryMove = MoveState.TURNPARALLELTOWALL;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVETOWHITELINE:
                lookingForWhiteLine = true;
                moveStraight(30.0, slowSpeed);
                currentMove = MoveState.STARTMOVE;
                telemetryMove = MoveState.DRIVETOWHITELINE;
                nextMove = MoveState.DRIVEPASTWHITELINE;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVEPASTWHITELINE:
                fColorSense.enableLed(false);
                lookingForWhiteLine = false;
                moveStraight(3.5, slowSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTOBEACON;
                telemetryMove = MoveState.DRIVEPASTWHITELINE;
                moveDelayTime = commonDelayTime;
                break;

            case TURNTOBEACON:
                moveTurn(90.0, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DRIVETOBEACON;
                telemetryMove = MoveState.TURNTOBEACON;
                currentOmni = OmniCtlr.RETRACT;
                moveDelayTime = 1500;
                break;

            case DRIVETOBEACON:
                lookingWithUltraSense = true;
                targetReading = 35.0;
                moveStraight(30.0, slowSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.WINDOUTARM;
                telemetryMove = MoveState.DRIVETOBEACON;
                moveDelayTime = 1500;
                break;

            case WINDOUTARM:
                chinUp.setTargetPosition(-3360);
                chinUp.setPower(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DUMPCLIMBERS;
                telemetryMove = MoveState.WINDOUTARM;
                moveDelayTime = 1500;
                break;

            case DUMPCLIMBERS:
                lookingWithUltraSense = false;
                climberDumper.setPosition(0.75);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.JIGGLEB;
                telemetryMove = MoveState.DUMPCLIMBERS;
                moveDelayTime = commonDelayTime;
                break;

            case JIGGLEB:
                moveStraight(-4, 1.0);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.JIGGLEF;
                telemetryMove = MoveState.JIGGLEB;
                moveDelayTime = 500;
                break;

            case JIGGLEF:
                moveStraight(2, 1.0);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.READBEACON;
                telemetryMove = MoveState.JIGGLEF;
                moveDelayTime = 500;
                break;

            case READBEACON:
                if (redAlliance){
                    if (bColorSense.red() >= 1.0) {
                        redL = true;
                    } else {
                        redR = true;
                    }
                } else if (blueAlliance) {
                    if (bColorSense.blue() >= 1.0) {
                        blueL = true;
                    } else {
                        blueR = true;
                    }
                }
                currentMove = MoveState.WINDINARMPARTWAY;
                break;

            case WINDINARMPARTWAY:
                chinUp.setTargetPosition(-150);
                chinUp.setPower(0.5);
                currentMove = MoveState.DELAYSETTINGS;
                nextMove = MoveState.TURNTOBUTTON;
                telemetryMove = MoveState.WINDINARMPARTWAY;
//                currentOmni = OmniCtlr.REEXTEND;
                moveDelayTime = commonDelayTime;
                break;

            case TURNTOBUTTON:
                if (redL || blueL) {
                    moveTurn(13, slowSpeed);
                    beaconPushTurn = 13;
                    targetReading = 23;
                } else if (redR || blueR) {
                    moveTurn(-13, slowSpeed);
                    beaconPushTurn = -13;
                    targetReading = 19;

                }
                currentOmni = OmniCtlr.REEXTEND;
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.PUSHBUTTON;
                telemetryMove = MoveState.TURNTOBUTTON;
                moveDelayTime = commonDelayTime;
                break;

            case PUSHBUTTON:
                lookingWithUltraSense = true;
                moveStraight(16.0, slowSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.BACKUP;
                telemetryMove = MoveState.PUSHBUTTON;
                moveDelayTime = commonDelayTime;
                break;

            case BACKUP:
                lookingWithUltraSense = false;
                moveStraight(-14.0, slowSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.TURNTOFLOORGOAL;
                telemetryMove = MoveState.BACKUP;
                moveDelayTime = commonDelayTime;
                break;

            case TURNTOFLOORGOAL:
                moveTurn(77 - beaconPushTurn, turnSpeed);
                currentMove = MoveState.STARTTURN;
                nextMove = MoveState.DRIVEINFLOORGOAL;
                telemetryMove = MoveState.TURNTOFLOORGOAL;
                currentOmni = OmniCtlr.RERETRACT;
                moveDelayTime = commonDelayTime;
                break;

            case DRIVEINFLOORGOAL:
                moveStraight(60, fastSpeed);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
                telemetryMove = MoveState.DRIVEINFLOORGOAL;
                moveDelayTime = 10;
                break;

            case DONE:
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                telemetryMove = MoveState.DONE;
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
                    currentOmni = OmniCtlr.PRENOTMOVING;
                }
                break;

            case PRENOTMOVING:
                chosenOmni = OmniCtlr.NOTMOVING;
                currentOmni = OmniCtlr.NOTMOVING;
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
                nextOmni = chosenOmni;
                moveDelaytimeOmni = 5500;
                break;

            case RETRACT:
                leftOmniPinion.setPosition(0.0);
                rightOmniPinion.setPosition(0.0);
                currentOmni = OmniCtlr.DELAYSETTINGSOMNI;
                nextOmni = chosenOmni;
                moveDelaytimeOmni = 4000;
                break;

            case REEXTEND:
                leftOmniPinion.setPosition(1.0);
                rightOmniPinion.setPosition(1.0);
                currentOmni = OmniCtlr.DELAYSETTINGSOMNI;
                nextOmni = chosenOmni;
                moveDelayTime = 4000;
                break;

            case RERETRACT:
                leftOmniPinion.setPosition(0.0);
                rightOmniPinion.setPosition(0.0);
                currentOmni = OmniCtlr.DELAYSETTINGSOMNI;
                nextOmni = OmniCtlr.DONE;
                moveDelayTime = 5500;
                break;

            case DONE:
                leftOmniPinion.setPosition(0.5);
                rightOmniPinion.setPosition(0.5);
                break;
        }
        Color.RGBToHSV(fColorSense.red(), fColorSense.green(), fColorSense.blue(), hsvValues);

        telemetry.addData("left encoder", leftDrive.getCurrentPosition());
        telemetry.addData("right encoder", rightDrive.getCurrentPosition());
        telemetry.addData("current move", telemetryMove.toString());
        telemetry.addData("Elapsed Time", currentTime.time());
        telemetry.addData("ColorSensor", Integer.toString(bColorSense.alpha()));
        telemetry.addData("red", Integer.toString(bColorSense.red()));
        telemetry.addData("blue", Integer.toString(bColorSense.blue()));
        telemetry.addData("green", Integer.toString(bColorSense.green()));
        telemetry.addData("ultraSense", ultraSense.getUltrasonicLevel());
        telemetry.addData("redAlliance", redAlliance);


    }

    @Override
    public void stop () {
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        chinUp.setTargetPosition(0);
        chinUp.setPower(0.0);
    }
}
