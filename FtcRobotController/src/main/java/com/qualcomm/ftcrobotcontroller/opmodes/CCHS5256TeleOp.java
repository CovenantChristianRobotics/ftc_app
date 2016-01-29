package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

/**
 * Created by Robotics on 1/21/2016.
 */
public class CCHS5256TeleOp extends OpMode {
    enum ledControl {
        PREMATCH, START, ON, ENDGAME, BLINKOFF, BLINKON, DELAYSETTINGS, DELAY
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
    // Speed functions
    boolean speedUp;
    boolean slowDown;
    //StateMachine Options
    ledControl currentControl;
    ledControl nextControl;
    ElapsedTime endGameTime;
    //Delay Settings
    long delayUntil;
    long moveDelayTime;
    Date now;

    /**
     * Constructor
     */
    public CCHS5256TeleOp() {
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
        leftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        chinUp.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        chinUp.setPower(0.2);
        endGameLights.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        endGameLights.setPower(0.7);
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
        armLock.setPosition(0.5);
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
    }

    @Override
    public void loop() {

        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double hang = gamepad2.left_stick_y;
        float omniWheels = gamepad2.left_stick_y;
        float rightStickPos = -gamepad1.right_stick_y;
        float rightStickNeg = -gamepad1.right_stick_y;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        hang = Range.clip(hang, -1, 1);
        omniWheels = Range.clip(omniWheels, -1, 1);

        if (rightStickPos < 0) {
            rightStickPos = 0;
        } else if (rightStickPos >= 0) {
            rightStickPos = gamepad1.right_stick_y;
        }

        if (rightStickNeg > 0) {
            rightStickNeg = 0;
        } else if (rightStickNeg <= 0) {
            rightStickNeg = gamepad1.right_stick_y;
        }

        if (gamepad1.left_bumper && !slowDown) {
            slowDown = true;
        } else if (gamepad1.right_bumper && !speedUp) {
            speedUp = true;
        } else if (gamepad1.left_bumper && slowDown) {
            slowDown = false;
        } else if (gamepad1.right_bumper && speedUp) {
            speedUp = true;
        }

        if (speedUp) {
            left = (float) fast(left);
            right = (float) fast(right);
        } else if (slowDown) {
            left = (float) slow(left);
            right = (float) slow(right);
        } else if (gamepad1.dpad_up) {
            left = rightStickPos * -1;
            right = rightStickPos * -1;
        } else if (gamepad1.dpad_down) {
            left = rightStickNeg * -1;
            right = rightStickNeg * -1;
        } else if (gamepad1.dpad_right) {
            left = -0.1;
            right = 0.1;
            if (speedUp) {
                left = (double) fast(left);
                right = (double) fast(right);
            } else if (slowDown) {
                left = (double) slow(left);
                right = (double) slow(right);
            } else {
                left = left;
                right = right;
            }
        } else if (gamepad1.dpad_left) {
            left = 0.1;
            right = -0.1;
            if (speedUp) {
                left = (double) fast(left);
                right = (double) fast(right);
            } else if (slowDown) {
                left = (float) slow(left);
                right = (float) slow(right);
            }
        } else {
            left = (float) medium(left);
            right = (float) medium(right);
        }

        if (gamepad2.right_bumper) {
            chinUp.setPower(hang);
        } else {
            chinUp.setPower(0.0);
        }

//        if (gamepad2.a) {
//            armLock.setPosition(0.2292121569);
//        } else if (gamepad2.b) {
//            armLock.setPosition(0.5);
//        }

        if (gamepad2.a) {
            armLock.setPosition(0.76862746);
        } else if (gamepad2.b) {
            armLock.setPosition(0.5);
        }

        if (gamepad1.a) {
            leftTrigger.setPosition(0.1);
        } else if (gamepad1.b) {
            leftTrigger.setPosition(0.8);
        }

        if (gamepad1.x) {
            rightTrigger.setPosition(0.8);
        } else if (gamepad1.y) {
            rightTrigger.setPosition(0.1);
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);

        if (gamepad2.dpad_left) {
            leftOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
        } else if (gamepad2.dpad_right) {
            rightOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
        } else {
            leftOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
            rightOmniPinion.setPosition((gamepad2.right_stick_y / 2) + 0.5);
        }

//        if (gamepad1.b) {
//            leftPlow.setPosition(Range.clip((leftPlow.getPosition() + 0.01), 0.0, 1.0));
//            rightPlow.setPosition(Range.clip((rightPlow.getPosition() + 0.01), 0.0, 1.0));
//        } else if (gamepad1.a) {
//            leftPlow.setPosition(Range.clip((leftPlow.getPosition() - 0.01), 0.0, 1.0));
//            rightPlow.setPosition(Range.clip((rightPlow.getPosition() - 0.01), 0.0, 1.0));
//        }

        if (gamepad2.y) {
            climberDumper.setPosition(0.55);
        } else if (gamepad2.x) {
            climberDumper.setPosition(1.0);
        }

        switch (currentControl) {
            case DELAYSETTINGS:
                now = new Date();
                delayUntil = now.getTime() + moveDelayTime;
                currentControl = ledControl.DELAY;
                break;

            case DELAY:
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentControl = nextControl;
                }
                break;

            case PREMATCH:
                currentControl = ledControl.START;
                break;

            case START:
                endGameTime.reset();
                endGameLights.setPower(1.0);
                currentControl = ledControl.ON;
                break;

            case ON:
                if (endGameTime.time() > 90.0) {
                    currentControl = ledControl.ENDGAME;
                } else {
                    currentControl = ledControl.ON;
                }
                break;

            case ENDGAME:
                endGameLights.setPower(0.0);
                currentControl = ledControl.BLINKON;
                break;

            case BLINKON:
                endGameLights.setPower(1.0);
                moveDelayTime = 325;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BLINKOFF;
                break;

            case BLINKOFF:
                endGameLights.setPower(0.0);
                moveDelayTime = 175;
                currentControl = ledControl.DELAYSETTINGS;
                nextControl = ledControl.BLINKON;
                break;
        }

        telemetry.addData("LED", currentControl.toString());
        telemetry.addData("Elapsed Time", endGameTime.time());
//        telemetry.addData("l plow", leftPlow.getPosition());
//        telemetry.addData("r plow", rightPlow.getPosition());
        telemetry.addData("enc left", leftDrive.getCurrentPosition());
        telemetry.addData("enc right", rightDrive.getCurrentPosition());
        telemetry.addData("arm lock", armLock.getPosition());
        telemetry.addData("climber dumper", climberDumper.getPosition());
        telemetry.addData("slow down", slowDown);
        telemetry.addData("speed up", speedUp);

    }
    @Override
    public void stop () {
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.04, 0.07, 0.10, 0.13, 0.16, 0.19, 0.23,
        0.28, 0.34, 0.41, 0.49, 0.58, 0.68, 0.72, 0.75, 0.75};

        //get the corresponding index for the scaleInput array
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return  dScale;
    }

    double fast(double jVal) {
        //scale input * 4/3 (top speed 1.0)
        return ((4 / 3) * scaleInput(jVal));
    }

    double medium(double jVal) {
        //scale input * 1.0 (top speed = 0.75)
        return  scaleInput(jVal);
    }

    double slow(double jVal) {
        // scale input * 0.5 (top speed = .375)
        return (0.5 * scaleInput(jVal));
    }
}
