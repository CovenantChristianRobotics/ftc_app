package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;

/**
 * Created by cchsrobochargers on 12/10/15.
 */
public class CCHS4507TeleOp extends OpMode {

    DcMotorController driveTrainController;
    DcMotorController auxMotorController;
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor trackLifter;

    @Override
    public void init() {
        //Ints hardware
        driveTrainController = hardwareMap.dcMotorController.get("dtCtlr");
        auxMotorController = hardwareMap.dcMotorController.get("auxCtlr");
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorLeft = hardwareMap.dcMotor.get("motorL");
        trackLifter = hardwareMap.dcMotor.get("trkLftr");

        //set up motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        trackLifter.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        trackLifter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    @Override
    public void loop() {
        //getting the gamepad values on the y axis
        float right = gamepad1.right_stick_y;
        float left = gamepad1.left_stick_y;
        float lifterMotor = gamepad2.right_stick_y;

        motorRight.setPower(right);
        motorLeft.setPower(left);
        trackLifter.setPower(lifterMotor);

    }
}
