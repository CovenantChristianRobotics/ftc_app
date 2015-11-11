package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.util.Date;
import java.util.concurrent.TimeUnit;

/**
 * Created by Robotics on 10/15/2015.
 */
public class LeftRightDriveTestTeleOp extends OpMode {
    enum MoveState {START1, DELAY1, MOVE1, DELAY2, START2, DELAY3, MOVE2, DONE};

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;
    MoveState currentMove;
    long delayUntil;
    Date now;
    int leftTargetPos;
    int rightTargetPos;

    public LeftRightDriveTestTeleOp() {
    }

    int centimetersToCounts(double centimeters) {
        return (int) ((centimeters / (10.0 * Math.PI)) * 1440.0);
    }

    @Override
    public void init() {
        driveTrainController = hardwareMap.dcMotorController.get("dtController");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        currentMove = MoveState.START1;
    }

    @Override
    public void loop() {

        switch (currentMove) {
            case START1:
                leftTargetPos = motorLeft.getCurrentPosition() + centimetersToCounts(243.84);
                rightTargetPos = motorRight.getCurrentPosition() + centimetersToCounts(243.84);
                //motorLeft.setDirection(DcMotor.Direction.FORWARD);
                //motorRight.setDirection(DcMotor.Direction.FORWARD);
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
                currentMove = MoveState.MOVE1;
                break;

            case MOVE1:
                if (motorLeft.getCurrentPosition() >= leftTargetPos) {
                    motorLeft.setPower(0.0);
                }
                if (motorRight.getCurrentPosition() >= rightTargetPos) {
                    motorRight.setPower(0.0);
                }
                if (motorLeft.getCurrentPosition() >= leftTargetPos && motorRight.getCurrentPosition() >= rightTargetPos) {
                    currentMove = MoveState.DELAY2;
                    now = new Date();
                    delayUntil = now.getTime() + 1000;
                }
                break;

            case DELAY2:
                now = new Date();
                if (now.getTime() >= delayUntil) {
                    currentMove = MoveState.START2;
                }
                break;

            case START2:
                leftTargetPos = motorLeft.getCurrentPosition() + centimetersToCounts(-243.84);
                rightTargetPos = motorRight.getCurrentPosition() + centimetersToCounts(-243.84);
                //motorLeft.setDirection(DcMotor.Direction.REVERSE);
                //motorRight.setDirection(DcMotor.Direction.REVERSE);
                motorLeft.setPower(-0.5);
                motorRight.setPower(-0.5);
                currentMove = MoveState.MOVE2;
                break;

            case MOVE2:
                if (motorLeft.getCurrentPosition() <= leftTargetPos) {
                    motorLeft.setPower(0.0);
                }
                if (motorRight.getCurrentPosition() <= rightTargetPos) {
                    motorRight.setPower(0.0);
                }
                if (motorLeft.getCurrentPosition() <= leftTargetPos && motorRight.getCurrentPosition() <= rightTargetPos) {
                    currentMove = MoveState.DONE;
                }
                break;

            case DONE:
                break;
        }
        if (gamepad1.start) {
            currentMove = MoveState.START1;
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float)motorRight.getCurrentPosition());
    }
        
    @Override
    public void stop() {

    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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