package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(group = "old")
public class RightPathAuto extends LinearOpMode {
    // robot with drive
    BaseRobot robot = new BaseRobot();
    //opencv
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;
    // other
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // OPEN CV
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new ZoneDetectionPipeline(telemetry, Fields.subRectX, Fields.subRectY, Fields.subRectWidth, Fields.subRectHeight);
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

        // Build Trajectories
        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(61, -54, Math.toRadians(0)))
                .build();
        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineTo(new Vector2d(62, -10))
                .addTemporalMarker(1.2, this::resetLift)
                .build();
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineTo(new Vector2d(65, -10))
                .build();
        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineToSplineHeading(new Pose2d(60, -15, Math.toRadians(45)))
                .build();
        Trajectory five = drive.trajectoryBuilder(four.end())
                .lineToSplineHeading(new Pose2d(62, -10, Math.toRadians(0)))
                .splineTo(new Vector2d(65, -10), 0)
                .addTemporalMarker(0.5, this::liftConeStack)
                .build();
        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(135)))
                .build();
        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineToSplineHeading(new Pose2d(35, -6, Math.toRadians(135)))
                .build();
        Trajectory eight = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(90)))
                .lineTo(new Vector2d(36, -10))
                .build();

        // Zone trajs
        Trajectory zone3 = drive.trajectoryBuilder(eight.end())
                .lineTo(new Vector2d(69, -12))
                .build();
        Trajectory zone1 = drive.trajectoryBuilder(eight.end())
                .lineTo(new Vector2d(2.3, -11.3))
                .build();

        /*
        Trajectory two = drive.trajectoryBuilder(one.end())
                .forward(-5)
                .build();
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineToSplineHeading(new Pose2d(54, -9, Math.toRadians(35)))
                .build();
        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineTo(new Vector2d(58, -5))
                .build();
        Trajectory five = drive.trajectoryBuilder(four.end())
                .forward(-1.5)
                .build();
        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineTo(new Vector2d(55, -7))
                .build();
        Trajectory seven = drive.trajectoryBuilder(six.end())
                        .forward(-1.5).build();

         */

        waitForStart();

        if (isStopRequested()) return;

        robot.closeClaw();
        delay(0.15);

        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        // NEW STUFF
        drive.followTrajectory(one);
        openClaw();
        liftSlightly();
        delay(0.5);
        drive.followTrajectory(two);
        liftConeStack();
        drive.followTrajectory(three);
        closeClaw();
        liftSmallGoal();
        drive.followTrajectory(four);
        openClaw();
        drive.followTrajectory(five);
        closeClaw();




        // AUTO CODE
        /*
        delay(0.15);
        drive.followTrajectory(terminal);
        openClaw();
        liftSlightly();
        delay(1);
        drive.followTrajectory(goBack);
        drive.followTrajectory(toConeStack);
        liftConeStack();
        delay(0.5);
        drive.followTrajectory(intoConeStack);
        closeClaw();
        delay(0.5);
        drive.followTrajectory(backUpALittle);
        liftSmallGoal();
        delay(2);
        drive.followTrajectory(depositSecond);
        openClaw();
        delay(0.5);
        liftConeStack();
        openClaw();
        delay(1);
        closeClaw();
        drive.followTrajectory(backUpAgain);
        liftOut();
        delay(1);

         */



        if (zone == 1) {
        }
        if (zone == 2) {
        } else {
            // zone 3
        }
    }
    // Auto robot functions
    public void liftHighGoal(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
            armRunTo(Fields.armForwardHigh);
        }
        delay(3);
    }
    public void liftOut() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.sliderBackwardsHigh);
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack);
        armRunTo(Fields.armConeStack);
    }
    public void liftSmallGoal() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.armBackwardsLow);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderForwardHigh);
    }
    public void openClaw() {
        robot.rightClaw.setPosition(Fields.rightClawDeliver);
        robot.leftClaw.setPosition(Fields.leftClawDeliver);
        delay(1);
    }
    // TODO this is the part that tips the entire robot over
    public void clearLift() {

        sliderRunTo(Fields.sliderForwardLow);
        delay(1);
        armRunTo(Fields.armBackwardsHigh, Fields.armSpeed);
        delay(1);
    }
    public void resetLift() {
        armRunTo(Fields.armGround);
        sliderRunTo(Fields.sliderGround);
        delay(1.5);
    }
    public void closeClaw() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
        delay(.5);
    }

    //helper functions
    private void armRunTo(int position){
        armRunTo(position, 1);
    }
    private void armRunTo(int position, double power){
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }
    private void sliderRunTo(int position){
        sliderRunTo(position, 1);
    }
    private void sliderRunTo(int position, double power){
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(power);
        robot.sideSlider.setTargetPosition(position);
        robot.sideSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sideSlider.setPower(power);
    }
    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t && !isStopRequested()) {
            telemetry.addLine("something");
        }
    }
}

