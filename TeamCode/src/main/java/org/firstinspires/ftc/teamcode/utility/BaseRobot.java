package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    // Drivetrain Reference
    SampleMecanumDrive drive;

    // Intake Motor
    public DcMotor intake;
    // Carousel Motor
    public DcMotor carousel;
    // Linear Slider Motor
    public DcMotor slider;
    // Linear Slider Deposit Bucket Servo
    public Servo bucket;
    public Servo intakeBar;
    public Servo capstoneArm;

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
    public void init(HardwareMap ahwMap, boolean RUN_USING_ENCODERS) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        // Initialize RoadRunner Sample Mecanum Drive
        drive = new SampleMecanumDrive(hwMap);

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        carousel = hwMap.dcMotor.get("carousel");
        intake = hwMap.dcMotor.get("intake");
        slider = hwMap.dcMotor.get("slider");
        bucket = hwMap.servo.get("bucket");
        intakeBar = hwMap.servo.get("intakeBar");
        capstoneArm = hwMap.servo.get("capstone");
        intake.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Enable Slider for Arm Run Code
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}