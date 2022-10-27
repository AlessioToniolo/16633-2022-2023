package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    // Drivetrain Reference
    public SampleMecanumDrive drive;

    // Linear Slider Motor
    public DcMotor slider;

    // Virtual Four Bar Motor;
    public DcMotor arm;
    public Servo leftClaw;
    public Servo rightClaw;

    // Local OpMode members
    HardwareMap hwMap;

    //IMU Fields
    public  BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    double previousHeading = 0;
    double integratedHeading = 0;
    private final ElapsedTime period = new ElapsedTime();

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
        slider = hwMap.dcMotor.get("slider");
        arm = hwMap.dcMotor.get("arm");

        leftClaw = hwMap.servo.get("leftclaw");
        rightClaw = hwMap.servo.get("rightclaw");




        // Initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Enable Slider & V4B for Arm Run Code
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void driveStraightInches(double speed,
                                    double inches,
                                    double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1;

        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = drive.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = drive.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = drive.leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = drive.rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            //
            drive.leftFront.setTargetPosition(newLeftFrontTarget);
            drive.rightFront.setTargetPosition(newRightFrontTarget);
            drive.leftRear.setTargetPosition(newLeftRearTarget);
            drive.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            drive.leftFront.setPower(Math.abs(speed));
            drive.rightFront.setPower(Math.abs(speed));
            drive.leftRear.setPower(Math.abs(speed));
            drive.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (drive.leftFront.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && drive.rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            drive.leftFront.setPower(0);
            drive.rightFront.setPower(0);
            drive.leftRear.setPower(0);
            drive.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void strafeInches(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1 * (12.0/29);

        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = drive.leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = drive.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = drive.leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = drive.rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            //
            drive.leftFront.setTargetPosition(newLeftFrontTarget);
            drive.rightFront.setTargetPosition(newRightFrontTarget);
            drive.leftRear.setTargetPosition(newLeftRearTarget);
            drive.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            drive.leftFront.setPower(Math.abs(speed));
            drive.rightFront.setPower(Math.abs(speed));
            drive.leftRear.setPower(Math.abs(speed));
            drive.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time drive.left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (drive.leftFront.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && drive.rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            drive.leftFront.setPower(0);
            drive.rightFront.setPower(0);
            drive.leftRear.setPower(0);
            drive.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        deg = deg * -1;
        DcMotor leftFront = drive.leftFront;
        DcMotor rightFront = drive.rightFront;
        DcMotor leftRear = drive.leftRear;
        DcMotor rightRear = drive.rightRear;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);

            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }


}