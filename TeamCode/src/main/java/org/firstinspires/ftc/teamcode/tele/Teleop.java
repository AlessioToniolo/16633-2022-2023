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
import com.qualcomm.robotcore.hardware.Gamepad;
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
    boolean retracted = false;
    //speed variables
    double baseSpeed = .5;
    double speed = 0;
    double triggerSpeedModifier = 1;

    //booleans for coneStack guessing algorithm
     int coneStackPos = -1;//-1 is not in use, 0 is out of bounds for looping purposes 1 is 1 cone 2 is 2 cones and so on and so forth until 5 cones where it will then loop around to 1
    int lastConeStackPos = 3;//records the last conestack position used
    boolean isGuessing = false;


    //Bumpers and triggers
    boolean prevRBumper, prevRBumper2=false;
    boolean prevLBumper,prevLBumper2 = false;
    boolean prevRTrigger, prevRTrigger2 = false;
    boolean prevLTrigger, prevLTrigger2 = false;

    // TODO meet 1 new stuf
    boolean prevGrasperA, prevGrasperA2 = false;

    //buttons
    boolean prevY, prevY2 = false;
    boolean prevX, prevX2 = false;
    boolean prevB, prevB2 = false;
    boolean prevA, prevA2=false;
    //dpad
    boolean prevDUp, prevDUp2 = false;
    boolean prevDLeft, prevDLeft2 = false;
    boolean prevDRight, prevDRight2 = false;
    boolean prevDDown, prevDDown2 = false;
    //MISC
    boolean prevGuide, prevGuide2 = false;


    double sliderTargetPos = 0;
    double rightClimber = 0;
    double leftClimber = 0;
    double v4bPos = Fields.v4bIntake;

    int clawPos = 0;//0 closed, 1 outtake, 2 intake

    // TODO new stuff
    int grasperPos = 0; // 0 closed, 1 outtake, 2 intake

    ElapsedTime runtime = new ElapsedTime();

    //IMU
    // IMU Fields
    BNO055IMU imu = robot.imu;
    BNO055IMU.Parameters imuParameters = robot.imuParameters;
    double robotDegree;
    double gamepadDegree;
    double changeDegree = 90;
    boolean imuInitialized= false;







    private boolean preValueGuide;

    ColorfulTelemetry pen;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();


        robot.init(hardwareMap);
        initializeImuParameters();
        initializeImu();

        // TODO new servo stuff
        //robot.v4bIntake();
        robot.closeAirplane();

        robot.closeClaw();
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
    public void runEverythingElseLoop() {
        while (!isStopRequested() && opModeIsActive()) {
           lineBreak();
            pen.setColor(ColorfulTelemetry.Red).setBold(true).addLine("SPEED DATA🏃💨");
            pen.reset().addData("BaseSpeed", baseSpeed);
            pen.addData("speedModifier", triggerSpeedModifier);
            pen.addLine("Speed: " + speed);
            lineBreak();
            pen.setBold(true).setColor(ColorfulTelemetry.Blue).addLine("DRIVE INFO🧭").reset();

            String modeName = imuInitialized ? "FIELDCENTRIC" : "FIELDCENTRIC: IMU UNITIALIZED";
            pen.addLine("DRIVE MODE: " + modeName);
            pen.addLine("Robot Degree: " + robotDegree);
            pen.addLine("Gamepad Degree: " + gamepadDegree);

            // TODO new servo stuff

            if (gamepad1.dpad_left) {
                robot.releaseAirplane();
            } else if (gamepad1.dpad_right) {
                robot.closeAirplane();
            }

//            checkClaw();
            checkV4b();
            checkSlider();
            checkClimber();
            checkButtonPresets();

            recenterIMU();
            doTelemetry();

        }
    }
    /*
    public void checkClaw(){
        if(gamepad2.a && gamepad2.a != prevA2){
            if(clawPos ==0){clawPos=1;robot.openClaw();}
            else if(clawPos ==1){clawPos=2;robot.intakeClaw();}
            else if(clawPos ==2){clawPos =0;robot.closeClaw();}
        }
        prevA2 = gamepad2.a;
    }

     */

    // todo new
    public void checkGrasper() {
        if (gamepad2.a && gamepad2.a != prevGrasperA2) {
            if(grasperPos ==0){grasperPos=1;robot.openGrasper();}
            else if(grasperPos ==1){grasperPos=2;robot.intakeGrasper();}
            else if(grasperPos ==2){grasperPos =0;robot.closeGrasper();}
        }
    }

    public void checkV4b(){
        v4bPos += gamepad2.right_stick_y*.05;
        if(v4bPos >Fields.maxV4bPos)v4bPos=1;
        else if(v4bPos<0)v4bPos=0;

        robot.v4bServo.setPosition(v4bPos);
    }
    public void checkSlider(){
        sliderTargetPos += (gamepad2.left_stick_y*-1)*10;
        if(sliderTargetPos < Fields.sliderIntake)sliderTargetPos = Fields.sliderIntake;
        robot.sliderRunTo(sliderTargetPos, Fields.sliderPower);

    }
    public void checkButtonPresets(){
        if(gamepad2.y && gamepad2.y != prevY2){
            sliderTargetPos = Fields.sliderIntake;
            v4bPos = Fields.v4bIntake;
            clawPos=2;
            grasperPos=2;
            robot.goToIntake();
        }
        prevY2 = gamepad2.y;
        if(gamepad2.x && gamepad2.x != prevX2){

            sliderTargetPos = Fields.sliderOuttake;
            v4bPos = Fields.v4bDeposit;



            robot.goToOuttake();
        }
        prevX2 = gamepad2.x;
        if(gamepad2.b && gamepad2.b != prevB2){
            robot.releaseAirplane();
        }
        prevB2 = gamepad2.b;

        /*
        if(gamepad2.a && gamepad2.a != prevA2){
            if(clawPos ==0){clawPos=1;robot.openClaw();}
            else if(clawPos ==1){clawPos=2;robot.intakeClaw();}
            else if(clawPos ==2){clawPos =0;robot.closeClaw();}
        }
        prevA2 = gamepad2.a;

         */

        // TODo meet 1 new claw
        if (gamepad2.a && gamepad2.a != prevGrasperA2){
            if(grasperPos ==0){grasperPos=1;robot.openGrasper();}
            else if(grasperPos ==1){grasperPos=2;robot.intakeGrasper();}
            else if(grasperPos ==2){grasperPos =0;robot.closeGrasper();}
        }
    }
    public void checkClimber(){
        if(Math.abs(gamepad2.right_trigger-gamepad2.left_trigger)>.2){
            //reset the left and right positions to the maximum of the two position
            if(leftClimber != rightClimber){
                leftClimber = Math.max(leftClimber, rightClimber);
            }

            leftClimber += (gamepad2.right_trigger-gamepad2.left_trigger)*20;
            rightClimber = leftClimber;
        }
        if(Math.abs(gamepad2.left_stick_x) > .05 && Math.abs(gamepad2.left_stick_y)<.05){
            leftClimber += gamepad2.left_stick_x*20;
        }
        if(Math.abs(gamepad2.right_stick_x)>.05 && Math.abs(gamepad2.right_stick_y) < .05){
            rightClimber += gamepad2.right_stick_x * 20;
        }
        if(gamepad2.dpad_up && gamepad2.dpad_up != prevDUp2){
            robot.climberReset();
            leftClimber = 0;
            rightClimber = 0;
        }
        prevDUp2 = gamepad2.dpad_up;
        robot.climberRunTo(leftClimber, rightClimber);

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
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }
    public void checkSpeed(){

        /** OLD
        double triggerSpeedModifier = 1.0-gamepad1.left_trigger;//left trigger works like a brake
        if(triggerSpeedModifier ==0)triggerSpeedModifier=.1;
        speed = triggerSpeedModifier*baseSpeed;

        if(gamepad1.left_stick_button)speed = 1;//set speed to 1
         if(Math.abs(gamepad1.left_trigger)>.1){
            speed = 1;
        }


        if(gamepad1.right_stick_button||gamepad1.right_trigger>0)speed=.2;


        if(speed>1)speed=1;
        else if(speed<0)speed=0;
    **/

    }

    public void recenterIMU(){
        if(gamepad1.b && gamepad1.b != prevB){
            changeDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        prevB = gamepad1.b;
    }




    public void doTelemetry() {
        pen.addLine("V4b: " + v4bPos);
        pen.addLine("TArget Slider: " + sliderTargetPos);
        pen.addLine("Left Climber" + leftClimber + "Right Climber: "+rightClimber);
        pen.addLine("CLIMBER POS" + robot.getClimberPos());
        pen.addLine("Slider Pos: " + robot.slider.getCurrentPosition());
        pen.addLine(" ClAW pos: " + clawPos);
        pen.update();
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



    /**
     * Helper Methods for telemetry
     */
    private void lineBreak(){
        pen.addLine();
        pen.setColor("White").addLine("_________________" );
        pen.addLine();
    }



}
