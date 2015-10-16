package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 9/29/2015.
 */
public class CCHS5256TeleOp extends OpMode {


DcMotorController leftDrive;
DcMotorController rightDrive;
DcMotorController armController1;
DcMotorController armController2;
DcMotor leftDrive1;
DcMotor leftDrive2;
DcMotor rightDrive1;
DcMotor rightDrive2;
DcMotor armController1A;
DcMotor armController1B;
DcMotor armController2A;
DcMotor armController2B;
ServoController servoController1;
Servo servo1A;
Servo servo1B;
Servo servo1C;
Servo servo1D;
Servo servo1E;
Servo servo1F;

    /**
     * Constructor
     */
    public CCHS5256TeleOp() {

    }

    @Override
    public void init() {


        leftDrive = hardwareMap.dcMotorController.get("left_drive");
        rightDrive = hardwareMap.dcMotorController.get("right_drive");
        armController1 = hardwareMap.dcMotorController.get("arm_controller_1");
        armController2 = hardwareMap.dcMotorController.get("arm_controller_2");
        leftDrive1 = hardwareMap.dcMotor.get("left_drive_1");
        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2 = hardwareMap.dcMotor.get("left_drive_2");
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1 = hardwareMap.dcMotor.get("right_drive_1");
        rightDrive2 = hardwareMap.dcMotor.get("right_drive_2");
        armController1A = hardwareMap.dcMotor.get("arm_1A");
        armController1B = hardwareMap.dcMotor.get("arm_1B");
        armController2A = hardwareMap.dcMotor.get("arm_2A");
        armController2B = hardwareMap.dcMotor.get("arm_2B");
        servoController1 = hardwareMap.servoController.get("servo_controller_1");
        servo1A = hardwareMap.servo.get("servo1A");
        servo1B = hardwareMap.servo.get("servo1B");
        servo1C = hardwareMap.servo.get("servo1C");
        servo1D = hardwareMap.servo.get("servo1D");
        servo1E = hardwareMap.servo.get("servo1E");
        servo1F = hardwareMap.servo.get("servo1F");
        leftDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMotorChannelMode(1, DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMotorChannelMode(2, DcMotorController.RunMode.RUN_USING_ENCODERS);

    }
@Override
public void loop() {

float left = gamepad1.left_stick_y;
float right = gamepad1.right_stick_y;

left = Range.clip(left, -1, 1);
right = Range.clip(right, -1, 1);

left = (float)scaleInput(left);
right = (float)scaleInput(right);

leftDrive1.setPower(left);
leftDrive2.setPower(left);
rightDrive1.setPower(right);
rightDrive2.setPower(right);



telemetry.addData("enc_left_1", (float) leftDrive.getMotorCurrentPosition(1));
telemetry.addData("enc_left_2", (float) leftDrive.getMotorCurrentPosition(2));
telemetry.addData("enc_right_1", (float) rightDrive.getMotorCurrentPosition(1));
telemetry.addData("enc_right_2", (float) rightDrive.getMotorCurrentPosition(2));
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
