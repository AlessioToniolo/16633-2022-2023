package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@TeleOp
public class Teleop extends LinearOpMode {

    BaseRobot robot = new BaseRobot();
    //dpad stuff
    boolean prevDUp = false;
    boolean prevDDown = false;
    boolean prevDRight = false;
    boolean prevDLeft = false;

    //speed variables
    double baseSpeed = .5;
    double speed = 0;
    double triggerSpeedModifier = 1;
    boolean prevRBumper=false;
    boolean prevLBumper = false;
    boolean prevRStickButton = false;
    boolean prevLStickButton = false;

    //slider vars
    int sliderState = 0;
    boolean prevA=false;

    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //robot.drive.setPoseEstimate(new Pose2d(12,12));

        robot.dualSlider.runAsync();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            runSliderServo();
            checkDrive();
            //checkDPads();
            checkSlider();
            doTelemetry();
        }
        robot.dualSlider.stopAsync();


    }

    public void runSliderServo() {
        if (gamepad2.left_bumper) {
            pos(0.3);
        } else if (gamepad2.right_bumper) {
            pos(-0.3);
        }
    }

    private void pos(double value)  {
        robot.leftSliderServo.setPosition(-value);
        robot.rightSliderServo.setPosition(value);
    }

    public void checkSlider(){
        if(gamepad1.right_trigger > 0)robot.dualSlider.setTargetPosition(robot.dualSlider.getTargetPosition()+10);
        else if(gamepad1.left_trigger>0) robot.dualSlider.setTargetPosition(robot.dualSlider.getTargetPosition()-10);

        if(gamepad1.a && gamepad1.a!=prevA){
            robot.dualSlider.goToPos(sliderState);
            sliderState++;
        }
        prevA=gamepad1.a;

    }

    public void checkDPads(){
        if(gamepad1.dpad_up&&gamepad1.dpad_up!=prevDUp){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .forward(24)
                    .build());
        }
        prevDUp= gamepad1.dpad_up;

        if(gamepad1.dpad_down&&gamepad1.dpad_down!=prevDDown){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .back(24)
                    .build());
        }
        prevDDown= gamepad1.dpad_down;

        if(gamepad1.dpad_right&&gamepad1.dpad_right!=prevDRight){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeRight(24)
                    .build());
        }
        prevDRight= gamepad1.dpad_right;

        if(gamepad1.dpad_left&&gamepad1.dpad_left!=prevDLeft){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(24)
                    .build());
        }
        prevDLeft= gamepad1.dpad_left;

    }
    public void checkDrive(){

        if(gamepad1.right_bumper&& gamepad1.right_bumper!=prevRBumper){//increases base speed
            baseSpeed +=.1;
        }
        prevRBumper = gamepad1.right_bumper;

        if(gamepad1.left_bumper&& gamepad1.left_bumper!=prevLBumper){//decreases base speed
            baseSpeed -=.1;
        }
        prevLBumper = gamepad1.left_bumper;

        double triggerSpeedModifier = 1.0-gamepad1.left_trigger;//left trigger works like a brake
        speed = triggerSpeedModifier*baseSpeed;

        if(gamepad1.left_stick_button)speed = 1;//set speed to 1

        if(gamepad1.right_stick_button)speed=.2;


        if(speed>1)speed=1;
        else if(speed<0)speed=0;

        double leftX;
        double leftY;
        double rightX;

        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        double leftRearPower = leftY + leftX - rightX;
        double leftFrontPower = leftY - leftX - rightX;
        double rightRearPower = leftY - leftX + rightX;
        double rightFrontPower = leftY + leftX + rightX;

        telemetry.addData("leftRear", leftRearPower);

        robot.drive.leftFront.setPower(leftFrontPower*speed);
        robot.drive.leftRear.setPower(leftRearPower*speed);
        robot.drive.rightFront.setPower(rightFrontPower*speed);
        robot.drive.rightRear.setPower(rightRearPower*speed);
    }
    public void doTelemetry() {
        //telemetry.addLine("Positon:" + robot.drive.getPoseEstimate());
        telemetry.addLine("SliderPos:" + robot.dualSlider.getTargetPosition());
        telemetry.addLine("BaseSpeed: "+baseSpeed);
        telemetry.addLine("speedModifier: "+triggerSpeedModifier);
        telemetry.addLine("Speed: "+speed);
        telemetry.update();


    }
}
