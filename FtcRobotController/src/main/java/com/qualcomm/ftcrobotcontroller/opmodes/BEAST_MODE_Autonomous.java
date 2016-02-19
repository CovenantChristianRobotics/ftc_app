// CCHS 5256 Autonomous Software
// Run in Autonomous Mode of FTC Challenge 2015-2016
// Autonomous for FIRST ResQ challenge;

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
 * Created by Robotics on 2/16/2016.
 */
public class BEAST_MODE_Autonomous extends OpMode {
    // DC Motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor chinUp;
    DcMotor endGameLights;
    DcMotor debrisSwivel;
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
    Servo dumperDoor;
    Servo sweeper;
    // Sensors
    GyroSensor gyroSense;
    ColorSensor fColorSense;
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
    boolean lookingWithUltraSense;
    double targetReading;
    // State Machine Settings
    double lDiff = 0.08549020;
    double rDiff = -0.11372550;
    long delayUntil;
    long moveDelayTime;
    long commonDelayTime;
    long now;
    long nowOmni;
    long moveDelaytimeOmni;
    long nowPlow;
    long plowDelayUntil;
    long delayUntilOmni;
    int counter;
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

// THE MARK OF THE BEAST

    long the_mark_of_the_beast;

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

        degrees = (degrees * redBlue);

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
        // DC Motors
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        chinUp = hardwareMap.dcMotor.get("chinUp");
        endGameLights = hardwareMap.dcMotor.get("endGameLights");
        debrisSwivel = hardwareMap.dcMotor.get("blockDumper");
        // DC Motor Settings
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        chinUp.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        endGameLights.setPower(0.9);
        debrisSwivel.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
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
        sweeper = hardwareMap.servo.get("sweeper");
        dumperDoor = hardwareMap.servo.get("dumperDoor");
        // Servo Settings
        armLock.setPosition(0.25);
        climberDumper.setPosition(0.45);
        rightOmniPinion.setDirection(Servo.Direction.REVERSE);
        leftOmniPinion.setPosition(0.5);
        rightOmniPinion.setPosition(0.5);
        leftPlow.setPosition(0.078431375);
        rightPlow.setPosition(0.75);
        leftTrigger.setPosition(0.5);
        rightTrigger.setPosition(0.5);
        sweeper.setPosition(0.5);
        dumperDoor.setPosition(0.0);
        // Sensors
        gyroSense = hardwareMap.gyroSensor.get("gyroSense");
        fColorSense = hardwareMap.colorSensor.get("fCS");
        ultraSense = hardwareMap.ultrasonicSensor.get("ultraSense");
        // Switches
        nearMtnSwitch = hardwareMap.digitalChannel.get("nMtnSw");
        redBlueBeaconSwitch = hardwareMap.digitalChannel.get("rBSw");
        delayPotSwitch = hardwareMap.digitalChannel.get("dPotSw");
        thirdTileSwitch = hardwareMap.digitalChannel.get("tileSw");
        delayPotentiometer = hardwareMap.analogInput.get("delPot");
        //statemachine settings
        counter = 0;
        // Set Switch Flags
        if (redBlueBeaconSwitch.getState()) {   // WE ARE BLUE
            redBlue = -1.0;
            ultraSenseServo.setPosition(0.75);
            redAlliance = false;
            blueAlliance = true;
        } else {                                // WE ARE RED
            redBlue = 1.0;
            ultraSenseServo.setPosition(0.25);
            redAlliance = true;
            blueAlliance = false;
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
        delay = 0.0;
        delayRobot = false;

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
        firstMoveDist = 71.44;
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
        Log.i("delay", Double.toString(delay));
        // Calibrate Gyro

        // THE MARK OF THE BEAST

        the_mark_of_the_beast = 666;

        gyroSense.calibrate();
        while (gyroSense.isCalibrating()) {
        }
    }

    @Override
    public void loop(){

    }
}
