package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.util.Date;
import java.util.concurrent.TimeUnit;

/**
 * Created by Robotics on 10/15/2015.
 */
public class PracticeTeleOp extends OpMode {

    DcMotorController driveTrainController;
    DcMotor motorRight;
    DcMotor motorLeft;

    public PracticeTeleOp() {
    }

    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft = hardwareMap.dcMotor.get("motorL");
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    @Override
    public void loop() {

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        motorLeft.setPower(left);
        motorRight.setPower(right);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
    }

    @Override
    public void stop() {

    }
}