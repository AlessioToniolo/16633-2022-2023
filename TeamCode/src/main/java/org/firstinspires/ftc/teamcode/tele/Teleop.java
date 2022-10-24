package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;

import java.lang.reflect.Field;

@TeleOp
public class Teleop extends LinearOpMode {

    BaseRobot robot = new BaseRobot();

    //speed variables
    double baseSpeed = .5;
    double speed = 0;
    double triggerSpeedModifier = 1;
    boolean prevRBumper=false;
    boolean prevLBumper = false;

    //slider vars
    int sliderState = 0;
    boolean prevA=false;
    double armTargetPos = 0;
    double sliderTargetPos = 0;
    boolean closed = false;

    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;
    double robotDegree;
    double gamepadDegree;



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        robot.init(hardwareMap);
        //robot.leftClaw.setPosition(1);
        //robot.rightClaw.setPosition(1);
//        robot.arm.setTargetPosition(-100);
//        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.arm.setPower(.5);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";//new
        imuParameters.loggingEnabled = true;//used to be false
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            checkSpeed();
            lockedFieldCentricDrive();
            checkSlider();
            doTelemetry();
        }


    }

    public void checkSlider(){
        if(gamepad1.left_bumper) {
            sliderTargetPos+=10;
        }
        if(gamepad1.right_bumper) {
            sliderTargetPos-=10;
        }
        if(gamepad1.left_trigger > 0) {
            armTargetPos+=.2;

        } else if(gamepad1.right_trigger>0) {
            armTargetPos-=.2;

        }
        if(gamepad1.a && gamepad1.a != prevA){
            if(closed) {
                closed= false;
                robot.rightClaw.setPosition(1);
                robot.leftClaw.setPosition(1);
            }
            else{
                closed = true;
                robot.rightClaw.setPosition(0);
                robot.leftClaw.setPosition(0);
            }

        }
        prevA = gamepad1.a;

        if(armTargetPos < Fields.armMinimumTarget)armTargetPos = Fields.armMinimumTarget;
        else if(armTargetPos > Fields.armMaximumTarget)armTargetPos = Fields.armMaximumTarget;
        if(sliderTargetPos < Fields.sliderMinimumTarget)sliderTargetPos = Fields.sliderMinimumTarget;
        else if(sliderTargetPos > Fields.sliderMaximumTarget)sliderTargetPos = Fields.sliderMaximumTarget;
        robot.arm.setTargetPosition((int)armTargetPos);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.slider.setTargetPosition((int)sliderTargetPos);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(1);
        telemetry.addLine("armTargetPos: "+armTargetPos);
        telemetry.addLine("armEstimatedPos: "+robot.arm.getTargetPosition());

        telemetry.addLine("SliderTargetPOs: "+sliderTargetPos);
        telemetry.addLine("SliderEstimatePOs: "+robot.slider.getTargetPosition());


    }
    public void lockedFieldCentricDrive(){




        double leftStickX;
        double leftStickY;
        double rightStickX;

        // Strafer Mode
        leftStickX = gamepad1.left_stick_x;
        leftStickY = -gamepad1.left_stick_y;//inversed to match x y plane coords
        /**
         * This is the left stick
         *        -1
         *        \
         *        \
         -1_______\________ +1 so -gamepad1.leftStickY turns it into the coordinate plane
         *        \
         *        \
         *
         *        +1
         */
        rightStickX = gamepad1.right_stick_x;

        //converting gamepad turn input into a speed for the motors to interpet
        if(rightStickX<0)rightStickX=-speed;
        else if(rightStickX>0)rightStickX=speed;




        //get degree of the robot from the imu
        robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addLine("Robot Degree: "+robotDegree);

        //LOCKING SYSTEM: converts gamepad input into either up left down or right
        boolean driveY = Math.abs(leftStickY)>Math.abs(leftStickX)?true:false;//determine whether we want to move on x or y axis
            if (driveY) {
                leftStickX = 0;
                gamepadDegree = leftStickY > 0 ? 90 : 270;
            } else {
                leftStickY = 0;
                gamepadDegree = leftStickX > 0 ? 0 : 180;
            }

            //DISABLED: This would be for normal fieldcentric
        //compute degree of joystick using atan of y/x
        //gamepadDegree = Math.atan2(leftStickY,leftStickX); normal way of doing it


        telemetry.addLine("Gamepad Degree: "+gamepadDegree);


        double turnDegrees = gamepadDegree-robotDegree;//determine what heading relative to the robot we want to drive


        //x and y are doubles in the range [-1,1] which are just the cos and sin of the angle you want to drive
        double x = round(Math.cos(Math.toRadians(turnDegrees)))*speed;//find x and y using cos and sin and then multiply them by the speed
        double y = round(Math.sin(Math.toRadians(turnDegrees)))*speed;

        if(Math.abs(leftStickY) == 0 &&Math.abs(leftStickX)==0 ){//however if there is no joystick movement x and y are 0
            x=0;
            y=0;
        }

        //calculate power; copied from the Nebomusc Macanum Quad with changes to match our motor directions
        double leftRearPower = y - x + rightStickX;
        double leftFrontPower = y + x + rightStickX;
        double rightRearPower = y + x - rightStickX;
        double rightFrontPower = y - x - rightStickX;

        telemetry.addLine("");
        telemetry.addLine("DRIVE DATA");
        telemetry.addLine("______________________________________");
        telemetry.addData("leftRear", leftRearPower);
        telemetry.addData("leftFront", leftFrontPower);
        telemetry.addData("rightRear", rightRearPower);
        telemetry.addData("rightFront", rightFrontPower);


            robot.drive.leftFront.setPower(leftFrontPower);
            robot.drive.leftRear.setPower(leftRearPower);
            robot.drive.rightFront.setPower(rightFrontPower);
            robot.drive.rightRear.setPower(rightRearPower);


    }
    public void fieldCentricDrive(){




        double leftStickX;
        double leftStickY;
        double rightStickX;

        // Strafer Mode
        leftStickX = gamepad1.left_stick_x;
        leftStickY = -gamepad1.left_stick_y;//inversed to match x y plane coords
        /**
         * This is the left stick
         *        -1
         *        \
         *        \
         -1_______\________ +1 so -gamepad1.leftStickY turns it into the coordinate plane
         *        \
         *        \
         *
         *        +1
         */
        rightStickX = gamepad1.right_stick_x;

        //converting gamepad turn input into a speed for the motors to interpet
        if(rightStickX<0)rightStickX=-speed;
        else if(rightStickX>0)rightStickX=speed;




        //get degree of the robot from the imu
        robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addLine("Robot Degree: "+robotDegree);



        //compute degree of joystick using atan of y/x
        gamepadDegree = Math.atan2(leftStickY,leftStickX); //normal way of doing it


        telemetry.addLine("Gamepad Degree: "+gamepadDegree);


        double turnDegrees = gamepadDegree-robotDegree;//determine what heading relative to the robot we want to drive


        //x and y are doubles in the range [-1,1] which are just the cos and sin of the angle you want to drive
        double x = round(Math.cos(Math.toRadians(turnDegrees)))*speed;//find x and y using cos and sin and then multiply them by the speed
        double y = round(Math.sin(Math.toRadians(turnDegrees)))*speed;


        //calculate power; copied from the Nebomusc Macanum Quad with changes to match our motor directions
        double leftRearPower = y - x + rightStickX;
        double leftFrontPower = y + x + rightStickX;
        double rightRearPower = y + x - rightStickX;
        double rightFrontPower = y - x - rightStickX;

        telemetry.addLine("");
        telemetry.addLine("DRIVE DATA");
        telemetry.addLine("______________________________________");
        telemetry.addData("leftRear", leftRearPower);
        telemetry.addData("leftFront", leftFrontPower);
        telemetry.addData("rightRear", rightRearPower);
        telemetry.addData("rightFront", rightFrontPower);


        robot.drive.leftFront.setPower(leftFrontPower);
        robot.drive.leftRear.setPower(leftRearPower);
        robot.drive.rightFront.setPower(rightFrontPower);
        robot.drive.rightRear.setPower(rightRearPower);


    }


    public void checkSpeed(){
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
    }
    public void checkDrive(){

        double leftX;
        double leftY;
        double rightX;

        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = -1*gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        double leftRearPower = leftY - leftX - rightX;
        double leftFrontPower = leftY + leftX - rightX;
        double rightRearPower = leftY + leftX + rightX;
        double rightFrontPower = leftY - leftX + rightX;
        telemetry.addLine("");
        telemetry.addLine("DRIVE DATA");
        telemetry.addLine("______________________________________");
        telemetry.addData("leftRear", leftRearPower);
        telemetry.addData("leftFront", leftFrontPower);
        telemetry.addData("rightRear", rightRearPower);
        telemetry.addData("rightFront", rightFrontPower);

        robot.drive.leftFront.setPower(leftFrontPower*speed);
        robot.drive.leftRear.setPower(leftRearPower*speed);
        robot.drive.rightFront.setPower(rightFrontPower*speed);
        robot.drive.rightRear.setPower(rightRearPower*speed);
    }
    public void doTelemetry() {
        //telemetry.addLine("Positon:" + robot.drive.getPoseEstimate());
        telemetry.addLine("SliderPos:" + robot.slider.getTargetPosition());
        telemetry.addLine("BaseSpeed: "+baseSpeed);
        telemetry.addLine("speedModifier: "+triggerSpeedModifier);
        telemetry.addLine("Speed: "+speed);
        telemetry.update();


    }

    public void armRunTo(int position){
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.02);
    }
    public static double round(double in){
        return ((int)(in*1000))/1000.0;
    }
}
