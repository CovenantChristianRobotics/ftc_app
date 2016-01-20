// CCHS 5256 Autonomous Software
// Run in Autonomous Mode of FTC CHallenge 2015-2016
// Autonomous for FIRST ResQ challenge

package com.qualcomm.ftcrobotcontroller.opmodes;

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

/**
 * Created by Robotics on 1/19/2016.
 */
public class CCHS5256Autonomous extends OpMode {

    enum MoveState {
        STARTMOVE, MOVINGSTRAIGHT, STARTTURN, MOVINGTURN, DELAYSETTINGS, DELAY,
        INITIALIZEROBOT, CHOOSEMOVE, FIRSTMOVE, TURNDIAG, MOVEDIAG, TURNONCOLOREDLINE, FOLLOWLINE,
        TURNTOBEACON, DRIVETOBEACON, READBEACON, PUSHBUTTON, UNPUSHBUTTON, BACKUP,
        DUMPCLIMBERS,BACKUPFARTHER, TURNALONGLINE, DRIVEALONGLINE, TURNTOMOUNTAIN,
        DRIVETOMOUNTAIN, GOUPMOUNTAIN, PREPTELEOP, DONE
    }

    // DC Motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor chinUp;
    DcMotor endGameLights;
    // Servos
    Servo beaconPinion;
    Servo beaconPusher;
    Servo climberDumper;
    Servo ultraSenseServo;
    Servo leftOmniPinion;
    Servo rightOmniPinion;
    // Sensors
    GyroSensor gyroSense;
    ColorSensor colorSense;
    UltrasonicSensor ultraSense;
    // Switches
    DigitalChannel nearMtnSwitch;
    DigitalChannel redBlueBeaconSwitch;
    DigitalChannel delayPotSwitch;
    DigitalChannel thirdTileSwitch;
    AnalogInput delayPotentiometer;
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
    // State Machine Settings
    MoveState currentMove;
    MoveState nextMove;
    MoveState telemetryMove;
    long moveDelayTime;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    double turnSpeed;
    // Elapsed Time
    ElapsedTime currentTime;

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
        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        endGameLights.setPower(1.0);
        // Servos
        beaconPinion = hardwareMap.servo.get("beaconPinion");
        beaconPusher = hardwareMap.servo.get("beaconPusher");
        climberDumper = hardwareMap.servo.get("climber_dumper");
        ultraSenseServo = hardwareMap.servo.get("servoUltra");
        leftOmniPinion = hardwareMap.servo.get("lOmniPinion");
        rightOmniPinion = hardwareMap.servo.get("rOmniPinion");
        // Servo Settings
        beaconPinion.setPosition(0.5);
        beaconPusher.setPosition(0.3);
        climberDumper.setPosition(0.5);
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        // Sensors
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        colorSense = hardwareMap.colorSensor.get("bColorSense");
        ultraSense = hardwareMap.ultrasonicSensor.get("fUltraSense");
        // Switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
        delayPotSwitch = hardwareMap.digitalChannel.get("dPotSw");
        thirdTileSwitch = hardwareMap.digitalChannel.get("tileSw");
        delayPotentiometer = hardwareMap.analogInput.get("delPot");
        // Set Switch Flags
        if (redBlueBeaconSwitch.getState()) {   // WE ARE RED
            redBlue = 1.0;
            ultraSenseServo.setPosition(0.25);
            redAlliance = true;
            blueAlliance = false;
        } else {                                // WE ARE BLUE
            redBlue = -1.0;
            ultraSenseServo.setPosition(0.75);
            redAlliance = false;
            blueAlliance = true;
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

        if (delayPotSwitch.getState()) {        // WE DELAY
            delay = (delayPotentiometer.getValue() * (10000 / 1024));
            delayRobot = true;
        } else {                                // WE DON'T DELAY
            delay = 0.0;
            delayRobot = false;
        }

        if (thirdTileSwitch.getState()) {       // WE ARE ON THE THIRD TILE FROM THE MOUNTAIN CORNER
            firstMoveDist = 30.38;
            turnDiagDegrees = 45.0;
            moveDiagDist = 0.0;
            thirdTile = true;
            fourthTile = false;
        } else {                                // WE ARE ON THE FOURTH TILE FROM THE MOUNTAIN CORNER
            firstMoveDist = 0.0;
            turnDiagDegrees = 0.0;
            moveDiagDist = 30.38;
            thirdTile = false;
            fourthTile = true;
        }

        // State Machine Settings
        fastSpeed = 0.8;
        mediumSpeed = 0.5;
        slowSpeed = 0.2;
        turnSpeed = 0.6;
        // Elapsed Time
        currentTime = new ElapsedTime();
        // log switch positions
        Log.i("delay", Double.toString(delay));
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

        endGameLights.setPower(1.0);

        switch (currentMove) {
            //  WE USE THESE IN ALL MOVES
            case STARTMOVE:
                currentMove = MoveState.MOVINGSTRAIGHT;
                break;

            case MOVINGSTRAIGHT:
                currentMove = MoveState.DELAYSETTINGS;
                break;

            case STARTTURN:
                currentMove = MoveState.MOVINGTURN;
                break;

            case MOVINGTURN:
                currentMove = MoveState.DELAYSETTINGS;
                break;

            case DELAYSETTINGS:
                currentMove = MoveState.DELAY;
                break;

            case DELAY:
                currentMove = nextMove;
                break;

            // MOVES WE USE ONCE, IN A SEQUENCE
            case INITIALIZEROBOT:
                currentTime.reset();
                moveDelayTime = (long)delay;
                currentMove = MoveState.DELAYSETTINGS;

        }

        telemetry.addData("left encoder", leftDrive.getCurrentPosition());
        telemetry.addData("right encoder", rightDrive.getCurrentPosition());
        telemetry.addData("current move", telemetryMove.toString());
        telemetry.addData("Elapsed Time", currentTime.time());

    }

    @Override
    public void stop () {
    }


}
