package org.firstinspires.ftc.teamcode.proto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    public DcMotor rightSlider;
    public DcMotor leftSlider;

    /*
    public CRServo intakeTester1;
    public CRServo intakeTester2;
    */

    // Local OpMode members
    HardwareMap hwMap;
    SampleMecanumDrive drive;

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        drive = new SampleMecanumDrive(hwMap);


        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        rightSlider = hwMap.dcMotor.get("rightSlider");
        leftSlider = hwMap.dcMotor.get("leftSlider");
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
        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
