package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    boolean prevB=false;
    double armTargetPos = 0;
    double sliderTargetPos = 0;
    boolean closed = false;

    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;
    double robotDegree;
    double gamepadDegree;

    //dpad
    boolean prevDUp = false;
    boolean prevDLeft = false;
    boolean prevDRight = false;
    boolean prevDDown = false;

    //buttons booleand
    boolean prevY = false;



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

        // TODO RR
        robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        while (!isStopRequested() && opModeIsActive()) {
            checkSpeed();
            fieldCentricDrive();
            checkSlider();
            doTelemetry();
        }


    }

    public void checkDriveRR() {
        // Read pose
        Pose2d poseEstimate = robot.drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        robot.drive.setWeightedDrivePower(
            new Pose2d(
                    input.getX(),
                    input.getY(),
                    -gamepad1.right_stick_x
            )
        );

        telemetry.addData("heading", /*poseEstimate.getHeading()*/robot.drive.getRawExternalHeading());
    }

    public void checkSlider(){

        if(gamepad2.left_trigger > 0) {
            sliderTargetPos-=10;

        } else if(gamepad2.right_trigger>0) {
            sliderTargetPos+=10;
        }
        armTargetPos+=10*-gamepad2.left_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 10

        //safety
        if(armTargetPos < Fields.armMinimumTarget)armTargetPos = Fields.armMinimumTarget;
        else if(armTargetPos > Fields.armMaximumTarget)armTargetPos = Fields.armMaximumTarget;

        if(sliderTargetPos>Fields.sliderMaximumTarget) sliderTargetPos = Fields.sliderMaximumTarget;
        else if(sliderTargetPos<Fields.sliderMinimumTarget) sliderTargetPos=Fields.sliderMinimumTarget;

        checkDUp();
        checkDDown();
        checkY();



        //motor functionality
        armRunTo((int)armTargetPos);
        sliderRunTo((int)sliderTargetPos);
        

        telemetry.addLine("SLIDER STUFF:" );
        telemetry.addLine("__________________________________:" );
        telemetry.addLine("armTargetPos: "+armTargetPos);
        telemetry.addLine("armEstimatedPos: "+robot.arm.getTargetPosition());

        telemetry.addLine("SliderTargetPos: "+sliderTargetPos);
        telemetry.addLine("SliderEstimatePos: "+robot.slider.getTargetPosition());


        //claw stuff not yet implemented
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

        telemetry.addLine("DRIVE MODE: LOCKED FIELDCENTRIC");



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
        rightStickX*=speed;//Experimental
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



        telemetry.addLine("DRIVE MODE: FIELDCENTRIC");

        //get degree of the robot from the imu
        robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addLine("Robot Degree: "+robotDegree);



        //compute degree of joystick using atan of y/x
        gamepadDegree = Math.atan2(Math.toRadians(leftStickY),Math.toRadians(leftStickX)); //normal way of doing it
        gamepadDegree = Math.toDegrees(gamepadDegree);

        telemetry.addLine("Gamepad Degree: "+gamepadDegree);


        double turnDegrees = gamepadDegree-robotDegree;//determine what heading relative to the robot we want to drive



        //x and y are doubles in the range [-1,1] which are just the cos and sin of the angle you want to drive
        double x = round(Math.cos(Math.toRadians(turnDegrees)))*speed;//find x and y using cos and sin and then multiply them by the speed
        double y = round(Math.sin(Math.toRadians(turnDegrees)))*speed;

        if(Math.abs(leftStickY) == 0 &&Math.abs(leftStickX)==0 ){//however if there is no joystick movement x and y are 0
            x=0;
            y=0;
        }

        rightStickX*=speed;//Experimental

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

        /**if(gamepad1.y) {
            robot.drive.leftFront.setPower(leftFrontPower);
            robot.drive.leftRear.setPower(leftRearPower);
            robot.drive.rightFront.setPower(rightFrontPower);
            robot.drive.rightRear.setPower(rightRearPower);
        }else{
            robot.drive.leftFront.setPower(0);
            robot.drive.leftRear.setPower(0);
            robot.drive.rightFront.setPower(0);
            robot.drive.rightRear.setPower(0);
        }**/
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
        telemetry.addLine("DRIVE MODE: NORMAL");

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

    public void checkDUp(){
        if(gamepad2.dpad_up && gamepad2.dpad_up != prevDUp){
            if(sliderState==Fields.referenceGroundPickup){
                sliderTargetPos=Fields.sliderConeStackPickup;
                sliderState=Fields.referenceConeStackPickup;
            }
            else if(sliderState==Fields.referenceConeStackPickup){
                sliderTargetPos=Fields.sliderLowJunctionLevel;
                sliderState=Fields.referenceLowJunction;
            }
            else if(sliderState==Fields.referenceLowJunction){
                sliderTargetPos=Fields.sliderMidJunctionLevel;
                sliderState=Fields.referenceMiddleJunction;
            }
            else if(sliderState ==Fields.referenceMiddleJunction){
                sliderTargetPos=Fields.sliderHighJunctionLevel;
                sliderState=Fields.referenceHighJunction;
            }
            else if(sliderState == Fields.referenceHighJunction)
            {
                sliderTargetPos=Fields.sliderGroundPickup;
                sliderTargetPos=Fields.referenceGroundPickup;
            }
        }
        prevDUp= gamepad2.dpad_up;
    }
    public void checkDDown(){
        if(gamepad2.dpad_down && gamepad2.dpad_down != prevDDown){
            if(sliderState==Fields.referenceGroundPickup){
                sliderTargetPos=Fields.sliderHighJunctionLevel;
                sliderState=Fields.referenceHighJunction;
            }
            else if(sliderState==Fields.referenceConeStackPickup){
                sliderTargetPos=Fields.sliderGroundPickup;
                sliderState=Fields.referenceGroundPickup;
            }
            else if(sliderState==Fields.referenceLowJunction){
                sliderTargetPos=Fields.sliderConeStackPickup;
                sliderState=Fields.referenceConeStackPickup;
            }
            else if(sliderState ==Fields.referenceMiddleJunction){
                sliderTargetPos=Fields.sliderLowJunctionLevel;
                sliderState=Fields.referenceLowJunction;
            }
            else if(sliderState == Fields.referenceHighJunction)
            {
                sliderTargetPos=Fields.sliderMidJunctionLevel;
                sliderTargetPos=Fields.referenceMiddleJunction;
            }
        }
        prevDDown= gamepad2.dpad_down;
    }
    public void checkY(){
        if(gamepad2.y && gamepad2.y != prevY){
            armRunTo(Fields.armPickup);
            sliderRunTo(Fields.sliderGroundPickup);
            sliderState = Fields.referenceGroundPickup;
        }
        prevY = gamepad2.y;
    }
    public void doTelemetry() {
        //telemetry.addLine("Positon:" + robot.drive.getPoseEstimate());
        telemetry.addLine("_____________: ");
        telemetry.addLine("SPEED DATA:");

        telemetry.addLine("BaseSpeed: "+baseSpeed);
        telemetry.addLine("speedModifier: "+triggerSpeedModifier);
        telemetry.addLine("Speed: "+speed);




        telemetry.update();


    }

    public void armRunTo(int position){
        armRunTo(position, 1);
    }
    public void armRunTo(int position, double power){
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }
    public void sliderRunTo(int position){
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(1);
    }
    public static double round(double in){
        return ((int)(in*1000))/1000.0;
    }
}
