package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

/**
 * Created by Robotics on 2/2/2016.
 */
public class sweeperTeleop extends OpMode {
    enum sweeperControl {
        PREMATCH, PREIDLE, IDLE, LEFT, RIGHT, UP, DOWN, DUMP, UNDUMP, SWEEP, DONE
    }
    DcMotor blockDumper;
    Servo dumperDoor;
    Servo sweeper;
    sweeperControl currentSweeper;
    sweeperControl chooseSweeper;
    boolean up;
    boolean left;
    boolean down;
    boolean right;

    double rotations (double rotationDegrees) {
        return ((3 + (1/9) * rotationDegrees));
    }

    public sweeperTeleop() {
    }

    @Override
    public void init() {
        blockDumper = hardwareMap.dcMotor.get("blockDumper");
        blockDumper.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        dumperDoor = hardwareMap.servo.get("dumperDoor");
        sweeper = hardwareMap.servo.get("sweeper");
        dumperDoor.setPosition(0.0);
        sweeper.setPosition(0.0);
        currentSweeper = sweeperControl.PREMATCH;
        down = true;
        up = false;
        left = false;
        right = false;
    }

    @Override
    public void loop() {

        if (gamepad1.a)
        switch (currentSweeper) {

            case PREMATCH:
                currentSweeper = sweeperControl.PREIDLE;
                break;

            case PREIDLE:
                if (!blockDumper.isBusy()) {
                    chooseSweeper = sweeperControl.IDLE;
                    currentSweeper = sweeperControl.IDLE;
                }
                break;

            case IDLE:
                telemetry.addData("IDLE","");
                currentSweeper = chooseSweeper;
                break;

            case LEFT:
                if (left) {
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (down) {
                    blockDumper.setTargetPosition((int) rotations(270));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (up) {
                    blockDumper.setTargetPosition((int) rotations(90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (right) {
                    blockDumper.setTargetPosition((int) rotations(180));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                }
                break;

            case RIGHT:
                if (right) {
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (up) {
                    blockDumper.setTargetPosition((int) rotations(-90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (down) {
                    blockDumper.setTargetPosition((int) rotations(90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (left) {
                    blockDumper.setTargetPosition((int) rotations(-180));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                }
                break;

            case UP:
                if (up) {
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (left) {
                    blockDumper.setTargetPosition((int) rotations(-90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (right) {
                    blockDumper.setTargetPosition((int) rotations(90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (down) {
                    blockDumper.setTargetPosition((int) rotations(180));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                }
                break;

            case DOWN:
                if (down) {
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (left) {
                    blockDumper.setTargetPosition((int) rotations(-270));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (up) {
                    blockDumper.setTargetPosition((int) rotations(-180));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                } else if (right) {
                    blockDumper.setTargetPosition((int) rotations(-90));
                    blockDumper.setPower(0.5);
                    currentSweeper = sweeperControl.PREIDLE;
                }
                break;

            case DUMP:
                dumperDoor.setPosition(0.0);
                currentSweeper = sweeperControl.PREIDLE;
                break;

            case UNDUMP:
                dumperDoor.setPosition(0.0);
                currentSweeper = sweeperControl.PREIDLE;
                break;

            case SWEEP:
                sweeper.setPosition(0.5);
                currentSweeper = sweeperControl.PREIDLE;
                break;

            case DONE:
                break;

        }
    }

    @Override
    public void stop() {
    }
}
