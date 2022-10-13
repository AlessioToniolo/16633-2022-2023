package org.firstinspires.ftc.teamcode.proto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseRobot {
    public DcMotor slider1;
    public DcMotor slider2;

    /*
    public CRServo intakeTester1;
    public CRServo intakeTester2;
    */

    // Local OpMode members
    HardwareMap hwMap;

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap) {
        // Save Reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        slider1 = hwMap.dcMotor.get("slider1");
        slider2 = hwMap.dcMotor.get("slider2");
        /*
        intakeTester1 = hwMap.crservo.get("intakeTester1");
        intakeTester2 = hwMap.crservo.get("intakeTester2");
         */
        /*
        intake.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
         */

        // Enable Slider for Arm Run Code
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
