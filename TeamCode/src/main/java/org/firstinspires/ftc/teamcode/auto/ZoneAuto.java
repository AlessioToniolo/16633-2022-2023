package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class ZoneAuto extends LinearOpMode{
    BaseRobot robot = new BaseRobot();
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        webcamName = hardwareMap.get(WebcamName .class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new ZoneDetectionPipeline(telemetry, Fields.subRectX,Fields.subRectY,Fields.subRectWidth,Fields.subRectHeight);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(myPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        waitForStart();
        delay(1);
        double zone = ZoneDetectionPipeline.getZone();
        delay(1);
        camera.stopStreaming();
        camera.closeCameraDevice();

        straight();
        if(zone==1)left();
        else if(zone ==3)right();



        /**
         robot.strafeInches(1,36,5);
         delay(.5);
         robot.pointTurnDegrees(1,180,5);
         delay(.5);
         robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.slider.setTargetPosition(Fields.sliderHighJunctionLevel);
         robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.arm.setTargetPosition(Fields.armDepositForward);
         robot.arm.setPower(1);
         robot.slider.setPower(1);
         delay(2);
         robot.leftClaw.setPosition(Fields.leftClawDeliver);
         robot.rightClaw.setPosition(Fields.rightClawDeliver);
         delay(1);
         robot.leftClaw.setPosition(Fields.rightClawPickup);
         robot.rightClaw.setPosition(Fields.leftClawPickup);
         //set claw to open a;jl;oiewqoia;lkjdsa;lkjfdsa
         //set claw to close ;LASJFDKA;LJSDFKJJDSAF
         robot.arm.setTargetPosition(0);
         robot.slider.setTargetPosition(0);
         robot.arm.setPower(1);
         robot.slider.setPower(1);
         if(zone==1)
         {
         robot.strafeInches(1,-24,5);
         }
         else if(zone==3)
         {
         robot.strafeInches(1,24,5);
         }**/
    }
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
        }
    }
    public void straight(){
        robot.drive.leftFront.setPower(1);
        robot.drive.rightFront.setPower(1);
        robot.drive.leftRear.setPower(1);
        robot.drive.rightRear.setPower(1);
        delay(.4);

        robot.drive.leftFront.setPower(0);
        robot.drive.rightFront.setPower(0);
        robot.drive.leftRear.setPower(0);
        robot.drive.rightRear.setPower(0);
    }
    public void backward(){
        robot.drive.leftFront.setPower(-1);
        robot.drive.rightFront.setPower(-1);
        robot.drive.leftRear.setPower(-1);
        robot.drive.rightRear.setPower(-1);
        delay(.5);

        robot.drive.leftFront.setPower(0);
        robot.drive.rightFront.setPower(0);
        robot.drive.leftRear.setPower(0);
        robot.drive.rightRear.setPower(0);
    }
    public void right(){
        robot.drive.leftFront.setPower(1);
        robot.drive.rightFront.setPower(-1);
        robot.drive.leftRear.setPower(-1);
        robot.drive.rightRear.setPower(1);
        delay(.8);

        robot.drive.leftFront.setPower(0);
        robot.drive.rightFront.setPower(0);
        robot.drive.leftRear.setPower(0);
        robot.drive.rightRear.setPower(0);
    }
    public void left(){
        robot.drive.leftFront.setPower(-1);
        robot.drive.rightFront.setPower(1);
        robot.drive.leftRear.setPower(1);
        robot.drive.rightRear.setPower(-1);
        delay(.8);

        robot.drive.leftFront.setPower(0);
        robot.drive.rightFront.setPower(0);
        robot.drive.leftRear.setPower(0);
        robot.drive.rightRear.setPower(0);
    }
}
