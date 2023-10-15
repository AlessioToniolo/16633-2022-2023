package org.firstinspires.ftc.teamcode.utility;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    // Drivetrain Reference
    public SampleMecanumDrive drive;

    // Color Sensor
    public RevColorSensorV3 distanceSensor;

    // Linear Slider Motor
    public DcMotor slider;

    public Motor climber1;
    public Motor climber2;

    public MotorGroup climber;

    // Virtual Four Bar Motor;
    public Servo leftClaw;
    public Servo rightClaw;

    // TODO new servos
    public Servo v4bServo;
    public Servo airplaneShooter;



    // Local OpMode members
    HardwareMap hwMap;

    //IMU Fields
    public  BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    double previousHeading = 0;
    double integratedHeading = 0;
    private final ElapsedTime period = new ElapsedTime();

    // other
    ElapsedTime runtime = new ElapsedTime();

    public double currentOrientation = 0;


    //Mr Michaud BAse RObot Encoder values
    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 0.6;
    private double     TURN_SPEED                    = 0.5;

    public final double TICKS_PER_REV = 537.6;
    public final double MAX_RPM = 312;
    public double WHEEL_RADIUS = 1.88976; // in
    public double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public double TRACK_WIDTH = 16.34; // in

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap/*, boolean RUN_USING_ENCODERS*/) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        // Initialize RoadRunner Sample Mecanum Drive
        //drive = new SampleMecanumDrive(hwMap);

        // Initialize color sensor
        //distanceSensor = hwMap.get(RevColorSensorV3.class, "color");

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        slider = hwMap.dcMotor.get("slider");

        leftClaw = hwMap.servo.get("leftclaw");
        rightClaw = hwMap.servo.get("rightclaw");

        // TODO new servos
        v4bServo = hwMap.servo.get("v4bServo");
        airplaneShooter = hwMap.servo.get("airplaneShooter");

        climber1 = new Motor(hwMap, "climber", Motor.GoBILDA.RPM_312);
        climber2 = new Motor(hwMap, "climber2", Motor.GoBILDA.RPM_312);
        climber = new MotorGroup(climber1, climber2);


        // Initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Enable Slider & V4B for Arm Run Code
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setRunMode(Motor.RunMode.PositionControl);
        climber.setRunMode(Motor.RunMode.PositionControl);
        climber.resetEncoder();

    }


    public void closeClaw() {
        rightClaw.setPosition(Fields.rightClawClose);
        leftClaw.setPosition(Fields.leftClawClose);
    }
    public void openClaw(){
        rightClaw.setPosition(Fields.rightClawDeliver);
        leftClaw.setPosition(Fields.leftClawDeliver);
    }
    public void intakeClaw(){
        rightClaw.setPosition(Fields.rightClawPickup);
        leftClaw.setPosition(Fields.leftClawPickup);
    }


    // TODO new servo functions
    public void releaseAirplane() {
        airplaneShooter.setPosition(Fields.airplaneRelease);
    }
    public void closeAirplane() {
        airplaneShooter.setPosition(Fields.airplaneClosed);
    }
    public void v4bDeposit() { v4bServo.setPosition(Fields.v4bDeposit);}
    public void v4bIntake() {
        v4bServo.setPosition(Fields.v4bIntake);
    }

    public void climberRunTo(double position, double power){
        climber.setTargetPosition((int)position);
        climber.setRunMode(Motor.RunMode.PositionControl);
        climber.set(power);
    }

    public void sliderRunTo(double position, double power){
        slider.setTargetPosition((int)position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(power);
    }
    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t) {
        }
    }

    // Old AUTO
    public void forward(double inch, double power) {
        double leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget;

        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontTarget = drive.leftFront.getCurrentPosition() + inchesToTicks(inch);
        rightFrontTarget = drive.rightFront.getCurrentPosition() + inchesToTicks(inch);
        leftRearTarget = drive.leftRear.getCurrentPosition() + inchesToTicks(inch);
        rightRearTarget = drive.rightRear.getCurrentPosition() + inchesToTicks(inch);


        drive.leftFront.setTargetPosition((int) leftFrontTarget);
        drive.rightFront.setTargetPosition((int) rightFrontTarget);
        drive.leftRear.setTargetPosition((int) leftRearTarget);
        drive.rightRear.setTargetPosition((int) rightRearTarget);

        // Turn On RUN_TO_POSITION
        drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftFront.setPower(power);
        drive.rightFront.setPower(power);
        drive.leftRear.setPower(power);
        drive.rightRear.setPower(power);
        /**while ((drive.leftFront.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && drive.rightRear.isBusy() )) {
            // Wait for Sequence to complete
        }**/

        // Stop all motion;
        /**drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightRear.setPower(0);**/
    }

    public void turn(double inch, double power) {
        double leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget;

        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontTarget = drive.leftFront.getCurrentPosition() + inchesToTicks(inch);
        rightFrontTarget = drive.rightFront.getCurrentPosition() - inchesToTicks(inch);
        leftRearTarget = drive.leftRear.getCurrentPosition() + inchesToTicks(inch);
        rightRearTarget = drive.rightRear.getCurrentPosition() - inchesToTicks(inch);


        drive.leftFront.setTargetPosition((int) leftFrontTarget);
        drive.rightFront.setTargetPosition((int) rightFrontTarget);
        drive.leftRear.setTargetPosition((int) leftRearTarget);
        drive.rightRear.setTargetPosition((int) rightRearTarget);

        // Turn On RUN_TO_POSITION
        drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftFront.setPower(power);
        drive.rightFront.setPower(power);
        drive.leftRear.setPower(power);
        drive.rightRear.setPower(power);
    }

    public void strafe(double inch, double power) {
        double leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget;

        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontTarget = drive.leftFront.getCurrentPosition() + inchesToTicks(inch);
        rightFrontTarget = drive.rightFront.getCurrentPosition() - inchesToTicks(inch);
        leftRearTarget = drive.leftRear.getCurrentPosition() - inchesToTicks(inch);
        rightRearTarget = drive.rightRear.getCurrentPosition() + inchesToTicks(inch);


        drive.leftFront.setTargetPosition((int) leftFrontTarget);
        drive.rightFront.setTargetPosition((int) rightFrontTarget);
        drive.leftRear.setTargetPosition((int) leftRearTarget);
        drive.rightRear.setTargetPosition((int) rightRearTarget);

        // Turn On RUN_TO_POSITION
        drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftFront.setPower(power);
        drive.rightFront.setPower(power);
        drive.leftRear.setPower(power);
        drive.rightRear.setPower(power);
    }

    public double inchesToTicks(double inches) {
        return TICKS_PER_REV * WHEEL_RADIUS * 2 * Math.PI * inches;
    }







    private void wait(double delay, Telemetry telemetry){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        int filler = 5;
        while(t.seconds()<delay){
            telemetry.addLine("Seconds: " + t.seconds());
            telemetry.update();
            filler=5;
        }
        return;
    }




}