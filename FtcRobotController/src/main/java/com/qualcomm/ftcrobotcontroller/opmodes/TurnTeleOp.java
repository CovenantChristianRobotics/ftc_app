package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.Date;

/**
 * Created by Robotics on 10/27/2015.
 */
public class TurnTeleOp extends OpMode {
    /**
     * Created by Robotics on 10/27/2015.
     */
    enum MoveState {
        START1, DELAY, DELAY1, DELAY2, STARTMOVE, MOVING, START2, START3, DONE
    }

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    MoveState currentMove;
    MoveState nextMove;
    int rightTarget;
    int leftTarget;
    long delayUntil;
    Date now;

    public TurnTeleOp() {
    }

    int centimetersToCounts(double centimeters) {
        return (int) ((centimeters / (10.1 * Math.PI)) * 1120.0);
    }

    int degreesToCounts(double degrees) {
        return centimetersToCounts((31.5 / 90.0) * degrees);
    }

    @Override
    public void init() {
        driveTrainController = hardwareMap.dcMotorController.get("dtController");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorLeft     .setDirection(DcMotor.Direction.REVERSE);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        currentMove = MoveState.START1;
    }

    @Override
    public void loop() {

        switch (currentMove) {
            case START1:
                leftTarget = motorLeft.getCurrentPosition() + centimetersToCounts(80);
                motorLeft.setTargetPosition(leftTarget);
                rightTarget = motorRight.getCurrentPosition() + centimetersToCounts(80);
                motorRight.setTargetPosition(rightTarget);
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
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
                motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + degreesToCounts(90));
                motorRight.setTargetPosition(motorRight.getCurrentPosition() - degreesToCounts(90));
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
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
                motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + centimetersToCounts(80));
                motorRight.setTargetPosition(motorRight.getCurrentPosition() + centimetersToCounts(80));
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
                currentMove = MoveState.STARTMOVE;
                nextMove = MoveState.DONE;
                break;

            case DONE:
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                break;
        }
        if (gamepad1.start) {
            currentMove = MoveState.START1;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("TGTleft", (float) leftTarget);
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
        telemetry.addData("TGTright", (float) rightTarget);
    }

    @Override
    public void stop() {

    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
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

        return dScale;
    }
}
