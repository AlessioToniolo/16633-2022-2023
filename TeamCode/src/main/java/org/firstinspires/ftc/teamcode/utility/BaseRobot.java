package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    // Drivetrain Reference
    public SampleMecanumDrive drive;

    // Left Slider Motor
    public DcMotor leftSlider;
    // Right Slider Motor
    public DcMotor rightSlider;

    public DcMotor rightFront, rightRear, leftFront, leftRear;

    // Linear Slider Deposit Bucket Servo
    public Servo leftSliderServo;
    public Servo rightSliderServo;

    // Local OpMode members
    HardwareMap hwMap;

    //IMU Fields
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    double previousHeading = 0;
    double integratedHeading = 0;
    private final ElapsedTime period = new ElapsedTime();

    public double currentOrientation = 0;

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap/*, boolean RUN_USING_ENCODERS*/) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        // Initialize RoadRunner Sample Mecanum Drive
        drive = new SampleMecanumDrive(hwMap);

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        leftSlider = hwMap.dcMotor.get("leftSlider");
        rightSlider = hwMap.dcMotor.get("rightSlider");

        leftFront = hwMap.dcMotor.get("leftFront");
        rightFront = hwMap.dcMotor.get("rightFront");
        leftRear = hwMap.dcMotor.get("leftRear");
        rightRear = hwMap.dcMotor.get("rightRear");
        //leftSliderServo = hwMap.servo.get("leftSliderServo");
        //rightSliderServo = hwMap.servo.get("rightSliderServo");

        // Initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Enable Slider for Arm Run Code
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}