package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CCHSRobotics on 3/15/2016.
 */
public class AutonomousWriter1_0_0 extends OpMode {
    enum DriveMode {
        READ, WRITE
    }

    enum WriteMode {
        PRE1, WAITFORSTART1, WAITFOREND1, POST1, CALCULATEDIFF1, WRITE1, DELAYSETTINGS1, DELAY1,
        PRE2, WAITFORSTART2, WAITFOREND2, POST2, CALCULATEDIFF2, WRITE2, DELAYSETTINGS2, DELAY2,
        PRE3, WAITFORSTART3, WAITFOREND3, POST3, CALCULATEDIFF3, WRITE3, DELAYSETTINGS3, DELAY3,
        PRE4, WAITFORSTART4, WAITFOREND4, POST4, CALCULATEDIFF4, WRITE4, DELAYSETTINGS4, DELAY4,
        PRE5, WAITFORSTART5, WAITFOREND5, POST5, CALCULATEDIFF5, WRITE5, DELAYSETTINGS5, DELAY5,
        PRE6, WAITFORSTART6, WAITFOREND6, POST6, CALCULATEDIFF6, WRITE6, DELAYSETTINGS6, DELAY6,
        PRE7, WAITFORSTART7, WAITFOREND7, POST7, CALCULATEDIFF7, WRITE7, DELAYSETTINGS7, DELAY7,
        PRE8, WAITFORSTART8, WAITFOREND8, POST8, CALCULATEDIFF8, WRITE8, DELAYSETTINGS8, DELAY8,
    }

    DcMotor leftDrive;
    DcMotor rightDrive;
    DigitalChannel teleOpSw;
    DigitalChannel readWriteSw;
    int move1LDist;
    int move1RDist;
    int pre1Time;
    int post1Time;
    int move2LDist;
    int move2RDist;
    int pre2Time;
    int post2Time;
    int move3LDist;
    int move3RDist;
    int pre3Time;
    int post3Time;
    int move4LDist;
    int move4RDist;
    int pre4Time;
    int post4Time;
    int move5LDist;
    int move5RDist;
    int pre5Time;
    int post5Time;
    int move6LDist;
    int move6RDist;
    int pre6Time;
    int post6Time;
    int move7LDist;
    int move7RDist;
    int pre7Time;
    int post7Time;
    int move8LDist;
    int move8RDist;
    int pre8Time;
    int post8Time;
    boolean alreadyWritten;
    int preLeft1;
    int preLeft2;
    int preLeft3;
    int preLeft4;
    int preLeft5;
    int preLeft6;
    int preLeft7;
    int preLeft8;
    int preRight1;
    int preRight2;
    int preRight3;
    int preRight4;
    int preRight5;
    int preRight6;
    int preRight7;
    int preRight8;
    int postLeft1;
    int postLeft2;
    int postLeft3;
    int postLeft4;
    int postLeft5;
    int postLeft6;
    int postLeft7;
    int postLeft8;
    int postLeft;
    int postRight;
    ElapsedTime currentTime;
    DriveMode currentMode;
    WriteMode currentWrite;

    public void init() {
        leftDrive = hardwareMap.dcMotor.get("motorL");
        rightDrive = hardwareMap.dcMotor.get("motorR");
        teleOpSw = hardwareMap.digitalChannel.get("tOSw");
        readWriteSw = hardwareMap.digitalChannel.get("rWSw");
    }

    public void loop() {
        switch (currentWrite) {

            case PRE1:
                preLeft1 = leftDrive.getCurrentPosition();
                preRight1 = rightDrive.getCurrentPosition();
                currentWrite = WriteMode.WAITFORSTART1;
                break;

            case WAITFORSTART1:
                if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right){
                    currentWrite = WriteMode.WAITFOREND1;
                }
                break;

            case WAITFOREND1:
                if((!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) && (!leftDrive.isBusy() && !rightDrive.isBusy())) {
                    currentWrite = WriteMode.POST1;
                }
                break;

            case POST1:
                postLeft = leftDrive.getCurrentPosition();
                postRight = rightDrive.getCurrentPosition();
                currentWrite = WriteMode.CALCULATEDIFF1;
                break;

            case CALCULATEDIFF1:
                move1LDist = postLeft - preLeft1;
                move1RDist = postRight - preRight1;
                currentWrite = WriteMode.WRITE1;
                break;

            case WRITE1:

                break;
        }

    }

    public void stop() {

    }
}
