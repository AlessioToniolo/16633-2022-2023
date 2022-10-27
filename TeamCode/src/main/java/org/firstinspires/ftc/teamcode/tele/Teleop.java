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
    double armTargetPos = 0;
    double sliderTargetPos = 0;
    boolean closed = false;
    //claw vars
    double leftClawPos = 0;
    double rightClawPos = 0;


    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;
    double robotDegree;
    double gamepadDegree;
    double changeDegree = 0;
    boolean imuInitialized= false;

    //dpad
    boolean prevDUp = false;
    boolean prevDLeft = false;
    boolean prevDRight = false;
    boolean prevDDown = false;


    //buttons booleand
    boolean prevY = false;
    boolean prevX = false;
    boolean prevB= false;
    boolean prevA2=false;
    boolean prevA=false;



    //arm state vars
    int armState = 0;//0=pickup 1=forwardDeliver, 2 =backwardDeliver



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        robot.init(hardwareMap);
        initializeImuParameters();
        initializeImu();
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
        telemetry.addLine("ready to Go");
        telemetry.update();
        waitForStart();

        // TODO RR
        //robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        while (!isStopRequested() && opModeIsActive()) {
            checkSpeed();
            fieldCentricDrive();
            checkSlider();
            checkClaw();
            checkInitializeImu();
            doTelemetry();
        }


    }

    public void checkSlider(){
        //manual Control
        sliderTargetPos+=10*-gamepad2.left_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 10
        armTargetPos+=10*-gamepad2.right_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 10

        //safety
        if(armTargetPos < Fields.armMinimumTarget)armTargetPos = Fields.armMinimumTarget;
        else if(armTargetPos > Fields.armMaximumTarget)armTargetPos = Fields.armMaximumTarget;

        if(sliderTargetPos>Fields.sliderMaximumTarget) sliderTargetPos = Fields.sliderMaximumTarget;
        else if(sliderTargetPos<Fields.sliderMinimumTarget) sliderTargetPos=Fields.sliderMinimumTarget;

        //automatic controls
        checkDDownandUp();
        checkY();
        checkDRightandLeft();
        checkXB();

        //motor functionality
        armRunTo((int)armTargetPos, .75);
        sliderRunTo((int)sliderTargetPos, .75);
        

        telemetry.addLine("SLIDER STUFF:" );
        telemetry.addLine("__________________________________:" );
        telemetry.addLine("ARM :" );
        telemetry.addLine("______" );
        telemetry.addLine("armTargetPos: "+armTargetPos);
        telemetry.addLine("armEstimatedPos: "+robot.arm.getTargetPosition());
        telemetry.addLine("armState: "+armState);
        telemetry.addLine("Slider :" );
        telemetry.addLine("______" );
        telemetry.addLine("SliderTargetPos: "+sliderTargetPos);
        telemetry.addLine("SliderEstimatedPos: "+robot.slider.getTargetPosition());
        telemetry.addLine("SliderState: "+sliderState);
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

        telemetry.addLine("__________________________________");
        telemetry.addLine("DRIVE INFO");
        String modeName = imuInitialized?"FIELDCENTRIC":"FIELDCENTRIC: IMU UNITIALIZED";
        telemetry.addLine("DRIVE MODE: "+modeName);


            //get degree of the robot from the imu
            robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            robotDegree-=changeDegree;
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



        //calculate power; copied from the Nebomusc Macanum Quad with changes to match our motor directions
        double leftRearPower = y - x + rightStickX;
        double leftFrontPower = y + x + rightStickX;
        double rightRearPower = y + x - rightStickX;
        double rightFrontPower = y - x - rightStickX;

        telemetry.addLine("______________________________________");
        telemetry.addLine("MOTOR DATA");
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
        if(triggerSpeedModifier ==0)triggerSpeedModifier=.1;
        speed = triggerSpeedModifier*baseSpeed;


        if(gamepad1.left_stick_button)speed = 1;//set speed to 1

        if(gamepad1.right_stick_button||gamepad1.right_trigger>0)speed=.2;


        if(speed>1)speed=1;
        else if(speed<0)speed=0;
        telemetry.addLine("_____________: ");
        telemetry.addLine("SPEED DATA:");
        telemetry.addLine("BaseSpeed: "+baseSpeed);
        telemetry.addLine("speedModifier: "+triggerSpeedModifier);
        telemetry.addLine("Speed: "+speed);
    }
    public void checkClaw(){
        if(gamepad2.a && gamepad2.a != prevA2){
            if(closed) {
                closed = false;
                if (armTargetPos > 200) {
                    robot.rightClaw.setPosition(Fields.rightClawDeliver);
                    robot.leftClaw.setPosition(Fields.leftClawDeliver);

                }
                else {
                    robot.rightClaw.setPosition(Fields.rightClawPickup);
                    robot.leftClaw.setPosition(Fields.leftClawPickup);
                }


            }
            else{

                closed = true;
                robot.rightClaw.setPosition(Fields.rightClawClose);
                robot.leftClaw.setPosition(Fields.leftClawClose);
            }

        }
        prevA2 = gamepad2.a;
        telemetry.addLine("__________________________");
        telemetry.addLine("CLAW INFO");
        telemetry.addLine("LEFT: Position:"+robot.leftClaw.getPosition()+"Port:"+robot.leftClaw.getPortNumber());
        telemetry.addLine("Right: Position:"+robot.rightClaw.getPosition()+"Port:"+robot.rightClaw.getPortNumber());
    }
    public void checkInitializeImu(){
        if(gamepad1.a && gamepad1.a != prevA){
            changeDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        prevA = gamepad1.a;
    }
    public void checkDDownandUp(){

        if(gamepad2.dpad_down && gamepad2.dpad_down != prevDDown){
            sliderState--;
        }
        prevDDown= gamepad2.dpad_down;
        if(gamepad2.dpad_up && gamepad2.dpad_up != prevDUp){
            sliderState++;
        }
        prevDUp= gamepad2.dpad_up;

        //allows cycle to circle around
        if(sliderState ==5)sliderState = 0;
        else if(sliderState==-1)sliderState = 4;

        if(gamepad2.dpad_up||gamepad2.dpad_down) {//if dpad up or down is pressed then use the sliderState
            // to determine the sliderTarget pos
            if (sliderState == Fields.referenceGroundPickup)
                sliderTargetPos = Fields.sliderGroundPickup;
            else if (sliderState == Fields.referenceConeStackPickup)
                sliderTargetPos = Fields.sliderConeStackPickup;
            else if (sliderState == Fields.referenceLowJunction)
                sliderTargetPos = Fields.sliderLowJunctionLevel;
            else if (sliderState == Fields.referenceMiddleJunction)
                sliderTargetPos = Fields.sliderMidJunctionLevel;
            else if (sliderState == Fields.referenceHighJunction)
                sliderTargetPos = Fields.sliderHighJunctionLevel;
        }
    }
    public void checkDRightandLeft() {
        if (gamepad2.dpad_right && gamepad2.dpad_right != prevDRight) {
            armState++;//incrase the armState
        }
        prevDRight = gamepad2.dpad_right;
        if (gamepad2.dpad_left && gamepad2.dpad_left != prevDLeft) {
            armState--;//decrease the arm state
        }
        prevDLeft = gamepad2.dpad_left;

        if(gamepad2.dpad_right || gamepad2.dpad_left){//if either of the buttons are pressed, then update the armPos
            //IMPORTANT: WE are only setting instance target positions here because every loop the slider and arm are set
            // to these target positions and called to move to these positions in the checkSlider() function

            //arm State is 0,1, or 2; this allows armState to circle from 2 -> 0 or from 0 ->2 in case armState ever becomes -1 or 3
            if (armState == -1) armState = 4;
            else if (armState == 5) armState = 0;

            // 0 = PIckup; 1 = deposit forward; 2 = deposit backward
            if (armState == 0) armTargetPos = Fields.armPickup;
            else if (armState == 1) armTargetPos = Fields.armDepositForward;
            else if (armState == 2) armTargetPos = Fields.armDepostForwardsHigh;
            else if (armState == 3) armTargetPos = Fields.armDepostBackwardsHigh;
            else if(armState ==4) armTargetPos = Fields.armDepositBackwards;

            //if we want to deposit backwards; moves the slider up to at least the low junction
            if(armState==2 && sliderTargetPos<200){
                sliderState=Fields.referenceLowJunction;
                sliderTargetPos=Fields.sliderLowJunctionLevel;
            }

        }
    }
    public void checkY(){
        if(gamepad2.y && gamepad2.y != prevY){
            sliderState = Fields.referenceGroundPickup;
            sliderTargetPos=Fields.sliderGroundPickup;
            armState = Fields.referenceArmPickup;
            armTargetPos=Fields.armPickup;
            armRunTo(Fields.armPickup);
            sliderRunTo(Fields.sliderGroundPickup);
            robot.leftClaw.setPosition(Fields.leftClawPickup);
            robot.rightClaw.setPosition(Fields.rightClawPickup);


        }
        prevY = gamepad2.y;
    }
    public void checkXB(){
        if(gamepad2.x&& gamepad2.x!=prevX){
            sliderState=Fields.referenceHighJunction;
            sliderTargetPos=Fields.sliderHighJunctionLevel;
            armState=Fields.referenceArmForwardsHigh;
            armTargetPos=Fields.armDepostForwardsHigh;
        }
        prevX=gamepad2.x;
        if(gamepad2.b&& gamepad2.b!=prevB){
            sliderState=Fields.referenceHighJunction;
            sliderTargetPos=Fields.sliderHighJunctionLevel;
            armState=Fields.referenceArmBackwardsHigh;
            armTargetPos=Fields.armDepostBackwardsHigh;
        }
        prevB=gamepad2.b;

    }
    public void doTelemetry() {

        telemetry.update();


    }
    //helper functions
    public void armRunTo(int position){
        armRunTo(position, 1);
    }
    public void armRunTo(int position, double power){
        armTargetPos=position;
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }
    public void sliderRunTo(int position){
        sliderRunTo(position, 1);
    }
    public void sliderRunTo(int position, double power){
        sliderTargetPos=position;
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(power);
    }
    public static double round(double in){
        return ((int)(in*1000))/1000.0;
    }

    public  void initializeImu(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public  void closeImu(){
        imu.close();
    }

    public void initializeImuParameters(){
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";//new
        imuParameters.loggingEnabled = true;//used to be false
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }
}
