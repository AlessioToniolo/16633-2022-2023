package org.firstinspires.ftc.teamcode.tele;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
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
    boolean prevRBumper2=false;
    boolean prevLBumper2 = false;
    boolean prevRTrigger = false;

    //slider vars
    int sliderState = 0;
    double armTargetPos = 0;
    double sliderTargetPos = 0;
    int coneStackPos = -1;//-1 is not in use, 0 is out of bounds for looping purposes 1 is 5 cones, 2 is 4 cones, 3 is 3 cones, 4 is 2 cones, 5 is 1 cone
    //claw vars
    double leftClawPos = 0;
    double rightClawPos = 0;
    volatile boolean closed = false;

    boolean prevGuide = false;

    ElapsedTime runtime = new ElapsedTime();

    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;
    double robotDegree;
    double gamepadDegree;
    double changeDegree = 90;
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

    String armStateStr = "GROUND";
    String sliderStateStr = "GROUND";



    //arm state vars
    int armState = 0;//0=pickup 1=forwardDeliver, 2 =backwardDeliver

    boolean sliderTestMode = false;
    private double sliderSpeedModifier;
    private boolean preValueGuide;

    ColorfulTelemetry pen;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();


        robot.init(hardwareMap);
        initializeImuParameters();
        initializeImu();
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
        pen = new ColorfulTelemetry(telemetry);
        pen.addLine("ready to Go");
        pen.update();

        waitForStart();

        // TODO RR
        //robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        Thread driveLoop = new Thread(this::runDriveLoop);
        driveLoop.start();
        runEverythingElseLoop();


    }
    public void runDriveLoop(){
        while (!isStopRequested() && opModeIsActive()) {
            checkSpeed();
            fieldCentricDrive();
        }
    }
    public void runEverythingElseLoop(){
        while (!isStopRequested() && opModeIsActive()) {
            pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
            pen.setColor(ColorfulTelemetry.Red).setBold(true).addLine("SPEED DATA🏃💨");
            pen.reset().addData("BaseSpeed",baseSpeed);
            pen.addData("speedModifier", triggerSpeedModifier);
            pen.addLine("Speed: "+speed);
            pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
            pen.addLine();
            pen.addLine("__________________________________");
            pen.setBold(true).setColor(ColorfulTelemetry.Blue).addLine("DRIVE INFO🧭");
            pen.reset();

            String modeName = imuInitialized?"FIELDCENTRIC":"FIELDCENTRIC: IMU UNITIALIZED";
            pen.addLine("DRIVE MODE: "+modeName);
            pen.addLine("Robot Degree: "+robotDegree);
            pen.addLine("Gamepad Degree: "+gamepadDegree);
            pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
            pen.addLine();

            checkSlider();
            checkClaw();
            recenterIMU();
            //checkColor();
            doTelemetry();

        }
    }
    public void checkColor() {
        double currentDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

        if (currentDistance <= Fields.distanceToClose && sliderState==Fields.referenceArmGround) {
            closed = true;
            robot.rightClaw.setPosition(Fields.rightClawClose);
            robot.leftClaw.setPosition(Fields.leftClawClose);
        }

        //pen.addLine(String.valueOf(currentDistance));
    }

    public void checkSlider(){
        //manual Control
        if(gamepad2.left_stick_button){
            sliderTargetPos+=50*-gamepad2.left_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 1
            sliderSpeedModifier = .5;
        }
        else {
            sliderSpeedModifier=0;
            sliderTargetPos += 10 * -gamepad2.left_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 10
            armTargetPos += 10 * -gamepad2.right_stick_y;//if pressed up then will add between 0 and positive 10 if pressed down will dubsttract between 0 and 10
        }

        //safety
        if(armTargetPos < Fields.armMinimumTarget)armTargetPos = Fields.armMinimumTarget;
        else if(armTargetPos > Fields.armMaximumTarget)armTargetPos = Fields.armMaximumTarget;

        if(sliderTargetPos>Fields.sliderMaximumTarget) sliderTargetPos = Fields.sliderMaximumTarget;
        else if(sliderTargetPos<Fields.sliderMinimumTarget) sliderTargetPos=Fields.sliderMinimumTarget;


        if(gamepad2.guide && gamepad2.guide != prevGuide){
            if(sliderTestMode){
                sliderTestMode=false;
            }
            else{
                sliderTestMode=true;
            }
        }
        prevGuide=gamepad2.guide;
//        pen.addLine("PracticeMode: "+sliderTestMode);
//        pen.addLine();


        //automatic controls

        checkY();
        if(closed) {
            checkXB();
            checkBumpers();
        }
        checkDDownandUp();
        checkDRightandLeft();
        checkRightTrigger();
        //checkResetEncoderPosition();


        //motor functionality
        armRunTo((int)armTargetPos, Fields.armSpeed);
        sliderRunTo((int)sliderTargetPos, Fields.sliderSpeed+ sliderSpeedModifier);
        




    }
    public void checkResetEncoderPosition(){
        if(gamepad2.guide&&gamepad2.guide != preValueGuide){
            robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           double sliderPower = 0;
           double armPower = 0;
            while(true){
                sliderPower=gamepad2.left_stick_y;
                armPower=gamepad2.right_stick_y;
                robot.slider.setPower(sliderPower);
                robot.arm.setPower(armPower);
                if(gamepad1.a && gamepad2.a != prevA2){
                    break;
                }
            }
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        preValueGuide=gamepad2.guide;
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
         *          *        -1
         *          *        \
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

//        pen.addLine("__________________________________");
//        pen.addLine("DRIVE INFO");
//        String modeName = imuInitialized?"FIELDCENTRIC":"FIELDCENTRIC: IMU UNITIALIZED";
//        pen.addLine("DRIVE MODE: "+modeName);


            //get degree of the robot from the imu
            robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            robotDegree-=changeDegree;
//            pen.addLine("Robot Degree: "+robotDegree);

            //compute degree of joystick using atan of y/x
            gamepadDegree = Math.atan2(Math.toRadians(leftStickY),Math.toRadians(leftStickX)); //normal way of doing it
            gamepadDegree = Math.toDegrees(gamepadDegree);

//            pen.addLine("Gamepad Degree: "+gamepadDegree);
//        pen.addLine("_________________" );
//        pen.addLine();

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
//        pen.addLine();
//        pen.addLine("______________________________________");
//        pen.addLine("MOTOR DATA");
//        pen.addData("leftRear", leftRearPower);
//        pen.addData("leftFront", leftFrontPower);
//        pen.addData("rightRear", rightRearPower);
//        pen.addData("rightFront", rightFrontPower);
//        pen.addLine("_______________________________________" );
//        pen.addLine();
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
        pen.addLine();
        pen.setColor(ColorfulTelemetry.Black).addLine("__________________________");
        pen.setColor(ColorfulTelemetry.Orange).setBold(true).addLine("CLAW INFO");
        pen.reset();
        pen.addLine("LEFT: Position:"+robot.leftClaw.getPosition()+"Port:"+robot.leftClaw.getPortNumber());
        pen.addLine("Right: Position:"+robot.rightClaw.getPosition()+"Port:"+robot.rightClaw.getPortNumber());
        pen.addLine("Closed: " + closed);
        pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
        pen.reset().addLine();
    }
    public void recenterIMU(){
        if(gamepad1.a && gamepad1.a != prevA){
            changeDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        prevA = gamepad1.a;
    }
    public void checkDDownandUp(){
        if(sliderTestMode) {
            if (gamepad2.dpad_down && gamepad2.dpad_down != prevDDown) {
                sliderState--;
            }
            prevDDown = gamepad2.dpad_down;
            if (gamepad2.dpad_up && gamepad2.dpad_up != prevDUp) {
                sliderState++;
            }
            prevDUp = gamepad2.dpad_up;
            if(gamepad2.dpad_up||gamepad2.dpad_down){
                updateSliderStates();}
        }
        else{
            if (gamepad2.dpad_down && gamepad2.dpad_down != prevDDown) {
                sliderState--;
                armState--;
            }
            prevDDown = gamepad2.dpad_down;
            if (gamepad2.dpad_up && gamepad2.dpad_up != prevDUp) {
                sliderState++;
                armState++;
            }
            prevDUp = gamepad2.dpad_up;
            if(gamepad2.dpad_up||gamepad2.dpad_down) {
                updateArmStates();
                updateSliderStates();
            }
        }

        //allows cycle to circle around
        if(sliderState ==8)sliderState = 0;
        else if(sliderState==-1)sliderState = 7;


        pen.addLine();
        pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
        pen.setBold(true).setColor(ColorfulTelemetry.Green).addLine("Slider :" );
        pen.reset().addLine("SliderTargetPos: "+sliderTargetPos + "Estimaed: " + robot.slider.getTargetPosition());
        pen.addLine("SliderState: "+sliderStateStr);
        pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
        pen.reset().addLine();
    }
    public void checkDRightandLeft() {
        if(sliderTestMode) {
            if (gamepad2.dpad_right && gamepad2.dpad_right != prevDRight) {
                armState++;//incrase the armState
            }
            prevDRight = gamepad2.dpad_right;
            if (gamepad2.dpad_left && gamepad2.dpad_left != prevDLeft) {
                armState--;//decrease the arm state
            }
            prevDLeft = gamepad2.dpad_left;
        }

            //arm State is 0,1, or 2; this allows armState to circle from 2 -> 0 or from 0 ->2 in case armState ever becomes -1 or 3
            if (armState == -1) armState = 7;
            else if (armState == 8) armState = 0;
            if((gamepad2.dpad_left||gamepad2.dpad_right)&&sliderTestMode){
                updateArmStates();
            }
            pen.addLine();
            pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
            pen.setColor(ColorfulTelemetry.Purple).setBold(true).addLine("ARM :" );
            pen.reset().addLine("armTargetPos: "+armTargetPos + "Estimated " + robot.arm.getTargetPosition());
            pen.addLine("armState: "+armStateStr);
            pen.setColor(ColorfulTelemetry.Black).addLine("_________________" );
            pen.reset().addLine();


        if(!sliderTestMode){
            if(armState == Fields.referenceArmConeStack && sliderState == Fields.referenceSliderConeStack){
                if(coneStackPos==-1)coneStackPos=5;

                if(gamepad2.dpad_left && gamepad2.dpad_left != prevDLeft){
                    coneStackPos--;
                    updateConeStackPos();
                }
                
                if(gamepad2.dpad_right && gamepad2.dpad_right != prevDRight){
                    coneStackPos++;
                    updateConeStackPos();
                }
                if(coneStackPos == 0)coneStackPos=5;
                else if(coneStackPos == 6)coneStackPos=1;






            }
            else{
                coneStackPos=-1;
            }
        }
        prevDLeft = gamepad2.dpad_left;
        prevDRight = gamepad2.dpad_right;
        pen.addLine();
        pen.addLine("___________________");
        String coneStackPosString = "";
        if(coneStackPos==-1)coneStackPosString = "NOT IN USE";
        else coneStackPosString = coneStackPos + " Cones Left";
        pen.addLine("CONE STACK POS: " + coneStackPosString);
        pen.addLine("___________________");
        pen.addLine();




        
    }
    public void checkY(){
        if(gamepad2.y && gamepad2.y != prevY){

            sliderState = Fields.referenceSliderGround;
            sliderTargetPos=Fields.sliderGround;
            armState = Fields.referenceArmGround;
            armTargetPos=Fields.armGround;
            sliderRunTo(Fields.sliderGround, 1);
            armRunTo(Fields.armGround, Fields.armSpeed);
            delay(1);
            robot.leftClaw.setPosition(Fields.leftClawPickup);
            robot.rightClaw.setPosition(Fields.rightClawPickup);
            closed=false;
        }
        prevY = gamepad2.y;
    }
    public void checkXB(){
        if(gamepad2.x&& gamepad2.x!=prevX){
            sliderState=Fields.referenceSliderBackwardsLow;
            sliderTargetPos=Fields.sliderBackLow;
            armState=Fields.referenceArmBackwardsLow;
            armTargetPos=Fields.armBackwardsLow;
        }
        prevX=gamepad2.x;
        if(gamepad2.b&& gamepad2.b!=prevB){
            sliderState=Fields.referenceSliderBackwardsHigh;
            sliderTargetPos=Fields.sliderBackwardsHigh;
            armState=Fields.referenceArmBackwardsHigh;
            armTargetPos=Fields.armBackwardsHigh;
        }
        prevB=gamepad2.b;

    }
    public void checkBumpers(){
        if(gamepad2.right_bumper && gamepad2.right_bumper!=prevRBumper2){
            sliderState = Fields.referenceSliderBackwardsMid;
            sliderTargetPos=Fields.sliderBackMid;
            armState = Fields.referenceArmBackwardsMid;
            armTargetPos = Fields.armBackwardsMid;
        }
        prevRBumper2=gamepad2.right_bumper;
        if(gamepad2.left_bumper && gamepad2.left_bumper!=prevLBumper2){
            sliderState = Fields.referenceSliderForwardLow;
            sliderTargetPos=Fields.sliderForwardLow;
            armState = Fields.referenceArmForwardLow;
            armTargetPos = Fields.armForwardLow;
        }
        prevLBumper2=gamepad2.left_bumper;

    }
    public void checkRightTrigger(){
        if(gamepad2.right_trigger > 0 && gamepad2.right_trigger>0 != prevRTrigger){
            sliderTargetPos = Fields.sliderBeacon;
            armTargetPos = Fields.armBeacon;
            sliderState = Fields.referenceSliderGround;
            armState = Fields.referenceArmGround;

        }
        prevRTrigger = gamepad2.right_trigger>0;
    }
    public void doTelemetry() {

        pen.update();


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
        sliderRunTo(position, Fields.sliderSpeed);
    }
    public void sliderRunTo(int position, double power){

            sliderTargetPos = position;
            robot.slider.setTargetPosition(position);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(power);
            robot.sideSlider.setTargetPosition(position);
            robot.sideSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sideSlider.setPower(power);
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
    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t) {
        }
    }

    public void updateSliderStates(){
        String stateStr = "";
        // to determine the sliderTarget pos
        if (sliderState == Fields.referenceSliderGround) {
            sliderTargetPos = Fields.sliderGround;
            stateStr = "GROUND";
        }
        else if (sliderState == Fields.referenceSliderConeStack) {
            sliderTargetPos = Fields.sliderConeStack;
            stateStr = "CONE STACK";
        }
        else if (sliderState == Fields.referenceSliderForwardLow) {
            sliderTargetPos = Fields.sliderForwardLow;
            stateStr = "FORWARD LOW";
        }
        else if (sliderState == Fields.referenceSliderForwardMid) {
            sliderTargetPos = Fields.sliderForwardMid;
            stateStr="FORWARD MIDDLE";
        }
        else if (sliderState == Fields.referenceSliderForwardHigh) {
            sliderTargetPos = Fields.sliderForwardHigh;
            stateStr = "FORWARD HIGH";
        }
        else if (sliderState == Fields.referenceSliderBackwardsHigh) {
            sliderTargetPos = Fields.sliderBackwardsHigh;
            stateStr = "BACKWARD HIGH";
        }
        else if (sliderState == Fields.referenceArmBackwardsMid) {
            sliderTargetPos = Fields.sliderBackMid;
            stateStr="BACKWARD MIDDLE";
        }else if (sliderState == Fields.referenceSliderBackwardsLow) {
            sliderTargetPos = Fields.sliderBackLow;
            stateStr = "BACKWARD LOW";
        }
        sliderStateStr=stateStr;
    }
    public void updateArmStates(){
        //IMPORTANT: WE are only setting instance target positions here because every loop the slider and arm are set
        // to these target positions and called to move to these positions in the checkSlider() function
        String stateStr="";
        if (armState == Fields.referenceArmGround) {
            armTargetPos = Fields.armGround;
            stateStr = "GROUND";
        } else if (armState == Fields.referenceArmConeStack) {
            armTargetPos = Fields.armConeStack;
            stateStr = "CONE STACK";
        } else if (armState == Fields.referenceArmForwardLow) {
            armTargetPos = Fields.armForwardLow;
            stateStr = "FORWARD LOW";
        } else if (armState == Fields.referenceArmForwardMid) {
            armTargetPos = Fields.armForwardMid;
            stateStr = "FORWARD MIDDLE";
        } else if (armState == Fields.referenceArmForwardsHigh) {
            armTargetPos = Fields.armForwardHigh;
            stateStr = "FORWARD HIGH";
        } else if (armState == Fields.referenceArmBackwardsHigh) {
            armTargetPos = Fields.armBackwardsHigh;
            stateStr = "BACKWARD HIGH";
        } else if (armState == Fields.referenceArmBackwardsMid) {
            armTargetPos = Fields.armBackwardsMid;
            stateStr = "BACKWARD MIDDLE";
        }
        else if (armState == Fields.referenceArmBackwardsLow) {
            armTargetPos = Fields.armBackwardsLow;
            stateStr = "BACKWARD LOW";
        }
        armStateStr=stateStr;
    }
    public void updateConeStackPos(){
        if(coneStackPos != 1)armTargetPos = Fields.armConeStack;
        if(coneStackPos==5)sliderTargetPos=Fields.coneStack5;
        else if(coneStackPos ==4)sliderTargetPos = Fields.coneStack4;
        else if(coneStackPos == 3)sliderTargetPos = Fields.coneStack3;
        else if(coneStackPos == 2)sliderTargetPos = Fields.coneStack2;
        else if(coneStackPos == 1){sliderTargetPos = Fields.coneStack1;armTargetPos=Fields.armGround;}
    }
}
