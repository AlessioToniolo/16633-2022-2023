package org.firstinspires.ftc.teamcode.auto;

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

@Autonomous
public class GriddyAuto extends LinearOpMode {
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

        Trajectory terminal = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(61, -54, Math.toRadians(0)))
                .build();
        Trajectory goBack = drive.trajectoryBuilder(terminal.end())
                .forward(-5)
                .build();
        Trajectory toConeStack = drive.trajectoryBuilder(goBack.end())
                .lineToSplineHeading(new Pose2d(57, -12, Math.toRadians(48)))
                .build();
        Trajectory intoConeStack = drive.trajectoryBuilder(toConeStack.end())
                .lineTo(new Vector2d(59, 20))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.closeClaw();
        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        // AUTO CODE
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
        liftSmallGoal();
        delay(1);

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
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack);
        armRunTo(Fields.armConeStack);
    }
    public void liftSmallGoal() {
        sliderRunTo(Fields.sliderBackLow);
        armRunTo(Fields.armBackwardsLow);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderBackMid);
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

