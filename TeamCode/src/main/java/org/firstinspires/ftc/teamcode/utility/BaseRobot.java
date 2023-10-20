package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BaseRobot {
    // Drivetrain Reference
    //public SampleMecanumDrive drive;

    // Color Sensor
    public RevColorSensorV3 distanceSensor;

    // Linear Slider Motor
    public DcMotor slider;

    //public Motor climber1;
//    public Motor climber2;
    public DcMotor climber1;
    public DcMotor climber2;
    public MotorGroup climber;

    // Virtual Four Bar Motor;
    public Servo leftClaw;
    public Servo rightClaw;

    // TODO new servos
    public Servo v4bServo;
    public Servo airplaneShooter;

    // TODO NEW NEW NEW MEET 1 SERVO
    public Servo grasper;



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
    private double     COUNTS_PER_MOTOR_REV          = 10455 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 0.6;
    private double     TURN_SPEED                    = 0.5;

    public final double TICKS_PER_REV = 3.4;
    public final double MAX_RPM = 312;
    public double WHEEL_RADIUS = 1.88976; // in
    public double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public double TRACK_WIDTH = 16.34; // in
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor rightFront;

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap/*, boolean RUN_USING_ENCODERS*/) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        // Initialize RoadRunner Sample Mecanum Drive
        //drive = new SampleMecanumDrive(hwMap);
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Initialize color sensor
        //distanceSensor = hwMap.get(RevColorSensorV3.class, "color");

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        slider = hwMap.dcMotor.get("slider");

        leftClaw = hwMap.servo.get("leftclaw");
        rightClaw = hwMap.servo.get("rightclaw");

        grasper = hwMap.servo.get("grasper");

        // TODO new servos
        v4bServo = hwMap.servo.get("v4bServo");
        airplaneShooter = hwMap.servo.get("airplaneShooter");
//        climber1 = new Motor(hwMap, "climber");
//        climber1.getCurrentPosition();
//        climber2 = new Motor(hwMap, "climber2");
//        climber2.getCurrentPosition();
//       climber = new MotorGroup(climber1, climber2);

        climber1 = hwMap.dcMotor.get("climber");
        climber2 = hwMap.dcMotor.get("climber2");
        climber1.setDirection(DcMotorSimple.Direction.REVERSE);

        climber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber1.setTargetPosition((int)0);
        climber2.setTargetPosition((int)0);



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
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        openGrasper();

    }



    public void closeClaw() {
        rightClaw.setPosition(Fields.rightClawClose);
        leftClaw.setPosition(Fields.leftClawClose);
    }


    // TODO new servo functions
    public void openGrasper() {
        grasper.setPosition(Fields.grasperOpen);
    }
    public void closeGrasper() {
        grasper.setPosition(Fields.grasperClose);
    }
    public void intakeGrasper() {
        grasper.setPosition(Fields.grasperIntake);
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
    public void climberReset(){
        climber1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber1.setTargetPosition(0);
        climber2.setTargetPosition(0);
        climber1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climber2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public String getClimberPos(){
        return "c1: " + climber1.getCurrentPosition() + "c2:" +climber2.getCurrentPosition();
    }
    public void climberRunTo(double leftClimberPos, double rightClimberPos){
        climber1.setTargetPosition((int)leftClimberPos);
        climber1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climber1.setPower(1);
        climber2.setTargetPosition((int)rightClimberPos);
        climber2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climber2.setPower(1);
    }

   public void goToOuttake(){
        v4bServo.setPosition(Fields.v4bMid);
        delay(.5);
        sliderRunTo(Fields.sliderOuttake, 1);
        delay(1.3);
       v4bServo.setPosition(Fields.v4bDeposit);

   }
    public void goToIntake(){
        closeClaw();
        v4bServo.setPosition(Fields.v4bMid);
        delay(.5);
        sliderRunTo(0, 1);
        delay(1);
        v4bServo.setPosition(Fields.v4bIntake);
        delay(1);
        openClaw();
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
    public void forward(double inch, double power, double timeoutS, Telemetry telemetry) {
        double leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget;


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontTarget = leftFront.getCurrentPosition() + inchesToTicks(inch);
        rightFrontTarget = rightFront.getCurrentPosition() + inchesToTicks(inch);
        leftRearTarget = leftRear.getCurrentPosition() + inchesToTicks(inch);
        rightRearTarget = rightRear.getCurrentPosition() + inchesToTicks(inch);


        leftFront.setTargetPosition((int) leftFrontTarget);
        rightFront.setTargetPosition((int) rightFrontTarget);
        leftRear.setTargetPosition((int) leftRearTarget);
        rightRear.setTargetPosition((int) rightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        period.reset();
        while ((period.seconds() < timeoutS) && (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
            // Wait for Sequence to complete
            telemetry.addLine("LeftFront" + leftFront.getTargetPosition() + " " + leftFront.getCurrentPosition() + " " + leftFront.isBusy());
            telemetry.addLine("leftRear" + leftRear.getTargetPosition() + " " + leftRear.getCurrentPosition() + " " + leftRear.isBusy());
            telemetry.addLine("rightFront" + rightFront.getTargetPosition() + " " + rightFront.getCurrentPosition() + " " + rightFront.isBusy());
            telemetry.addLine("rightRear" + rightRear.getTargetPosition() + " " + rightRear.getCurrentPosition() + " " + rightRear.isBusy());
            telemetry.addLine("Time" + period.seconds());
            telemetry.update();
        }


        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS, Telemetry telemetry) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        deg = deg * (90.0 / 270.0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            newLeftRearTarget =leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
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
            leftFront.setPower(speed);
            rightFront.setPower(speed);
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
                telemetry.addLine("LEFtFront" + leftFront.getTargetPosition() + " " + leftFront.getCurrentPosition() + " " + leftFront.isBusy());
                telemetry.addLine("leftRear" + leftRear.getTargetPosition() + " " + leftRear.getCurrentPosition() + " " + leftRear.isBusy());
                telemetry.addLine("rightFront" + rightFront.getTargetPosition() + " " + rightFront.getCurrentPosition() + " " + rightFront.isBusy());
                telemetry.addLine("rightRear" + rightRear.getTargetPosition() + " " + rightRear.getCurrentPosition() + " " + rightRear.isBusy());
                telemetry.addLine("Period" + period.seconds());
                telemetry.update();
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


    public void strafe(double inch, double power, double timeoutS) {
        double leftFrontTarget, rightFrontTarget, leftRearTarget, rightRearTarget;
        period.reset();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontTarget = leftFront.getCurrentPosition() + inchesToTicks(inch);
        rightFrontTarget = rightFront.getCurrentPosition() - inchesToTicks(inch);
        leftRearTarget = leftRear.getCurrentPosition() - inchesToTicks(inch);
        rightRearTarget = rightRear.getCurrentPosition() + inchesToTicks(inch);


        leftFront.setTargetPosition((int) leftFrontTarget);
        rightFront.setTargetPosition((int) rightFrontTarget);
        leftRear.setTargetPosition((int) leftRearTarget);
        rightRear.setTargetPosition((int) rightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        while ((period.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
            // Wait for Sequence to complete
            telemetry.addLine("LEFtFront" + leftFront.getTargetPosition() + " " + leftFront.getCurrentPosition() + " " + leftFront.isBusy());
            telemetry.addLine("leftRear" + leftRear.getTargetPosition() + " " + leftRear.getCurrentPosition() + " " + leftRear.isBusy());
            telemetry.addLine("rightFront" + rightFront.getTargetPosition() + " " + rightFront.getCurrentPosition() + " " + rightFront.isBusy());
            telemetry.addLine("rightRear" + rightRear.getTargetPosition() + " " + rightRear.getCurrentPosition() + " " + rightRear.isBusy());
            telemetry.addLine("Time" + period.seconds());
            telemetry.update();
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