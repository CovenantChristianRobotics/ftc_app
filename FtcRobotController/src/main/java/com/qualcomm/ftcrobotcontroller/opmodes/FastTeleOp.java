package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Robotics on 10/15/2015.
 */
public class FastTeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    String text;

    public FastTeleOp() {
    }

    @Override
    public void init() {

        motorLeft = hardwareMap.dcMotor.get("l");
        motorRight = hardwareMap.dcMotor.get("r");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        text = "JosB Pineapple";
    }

    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

//        if(left > 1 || left < -1) {
//            motorLeft.setPower(left);
//        } else {
//            motorLeft.setPowerFloat();
//        }
//
//        if (right > 1 || right < -1) {
//            motorRight.setPower(right);
//        } else {
//            motorRight.setPowerFloat();
//        }
        motorLeft.setPower(left);
        motorRight.setPower(right);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("ENCLeft", (float) motorLeft.getCurrentPosition());
        telemetry.addData("ENCRight", (float) motorRight.getCurrentPosition());
        telemetry.addData("Name", text.toUpperCase());
    }

    @Override
    public void stop() {

    }
}