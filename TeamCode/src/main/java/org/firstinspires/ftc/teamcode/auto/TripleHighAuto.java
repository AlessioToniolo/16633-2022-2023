package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class TripleHighAuto extends LinearOpMode {
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
                .lineTo(new Vector2d(36, -20), SampleMecanumDrive.getVelocityConstraint(56, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // FIRST DEPOSIT
        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToLinearHeading(new Pose2d(37.7, -4.5, Math.toRadians(132)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        /*
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineToLinearHeading(new Pose2d(36, -10, Math.toRadians(90)))
                .build();

         */
        Trajectory twoHalf = drive.trajectoryBuilder(two.end())
                .lineTo(new Vector2d(32, -13))
                .build();
        // SECOND PICKUP
        Trajectory three = drive.trajectoryBuilder(twoHalf.end())
                .lineToLinearHeading(new Pose2d(60.5, -11, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, ()->{
                    fastOpenClaw();
                    liftConeStack();
                })
                .build();
        Trajectory threeHalf = drive.trajectoryBuilder(three.end())
                .lineTo(new Vector2d(45, -7))
                .addTemporalMarker(0.1, () -> {
                    fastLiftHigh(false, 0.5);
                })/*
                .addTemporalMarker(1, () -> {
                    fastLiftHigher(true, 0.5);
                })*/
                .build();
        // SECOND DEPOSIT
        Trajectory four = drive.trajectoryBuilder(threeHalf.end())
                .lineToLinearHeading(new Pose2d(31.5, -2, Math.toRadians(-35)))
                .build();
        Trajectory fourHalf = drive.trajectoryBuilder(four.end())
                .lineTo(new Vector2d(33, -8))
                .build();
        // THIRD PICKUP
        Trajectory preFive = drive.trajectoryBuilder(fourHalf.end())
                .lineToLinearHeading(new Pose2d(45, -6, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, ()->{
                    fastOpenClaw();
                    liftConeStackLess();
                })
                .build();
        Trajectory five = drive.trajectoryBuilder(preFive.end())
                .lineTo(new Vector2d(61, -8))
                .build();
        Trajectory fiveHalf = drive.trajectoryBuilder(five.end())
                .lineTo(new Vector2d(45, -7))
                .addTemporalMarker(0.1, () -> {
                    fastLiftHigh(true, 0.5);
                })
                .build();
        // Third Deposit
        Trajectory six = drive.trajectoryBuilder(fiveHalf.end())
                .lineToLinearHeading(new Pose2d(32.5, -0.3, Math.toRadians(-35)))
                .build();

        // Zone trajs
        Trajectory zone3 = drive.trajectoryBuilder(six.end())
                .lineToLinearHeading(new Pose2d(69, -12, Math.toRadians(90)))
                .build();

        Trajectory zone1 = drive.trajectoryBuilder(six.end())
                .lineToLinearHeading(new Pose2d(2.3, -11.3, Math.toRadians(90)))
                .build();


        // Zone trajs
        /*
        Trajectory zone3 = drive.trajectoryBuilder(eight.end())
                .lineTo(new Vector2d(69, -12))
                .build();
        Trajectory zone1 = drive.trajectoryBuilder(eight.end())
                .lineTo(new Vector2d(2.3, -11.3))
                .build();
        */

        waitForStart();

        if (isStopRequested()) return;

        robot.closeClaw();

        // OpenCV Code
        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        // NEW STUFF
        drive.followTrajectory(one);
        fastLiftLower(false, 0.55);
        delay(0.2);
        drive.followTrajectory(two);
        openClaw();
        drive.followTrajectory(twoHalf);
        drive.followTrajectory(three);
        closeClaw();
        liftSlightly();
        delay(0.2);
        drive.followTrajectory(threeHalf);
        drive.followTrajectory(four);
        openClaw();
        drive.followTrajectory(fourHalf);
        drive.followTrajectory(preFive);
        drive.followTrajectory(five);
        closeClaw();
        liftSlightly();
        delay(0.2);
        // TODO
        drive.followTrajectory(fiveHalf);
        drive.followTrajectory(six);
        openClaw();


        if (zone == 1) {
            drive.followTrajectory(zone1);
        }
        if (zone == 3) {
            drive.followTrajectory(zone3);
        }
        resetLift();
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
    public void liftHighGoal(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh, power);
            armRunTo(Fields.armForwardHigh, power);
        }
        delay(3);
    }
    public void fastLiftHigh(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftHigh(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void fastLiftHigher(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh+50);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh+50);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftHigher(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh+50, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh+50, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }

    public void fastLiftLower(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-30);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh+30);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftLower(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-50, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh+50, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }


    public void liftOut() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.sliderBackwardsHigh);
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack+10);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStackLess() {
        sliderRunTo(Fields.sliderConeStack-100);
        armRunTo(Fields.armConeStack+50);
    }
    public void liftConeStackLessLess() {
        sliderRunTo(Fields.sliderConeStack-100);
        armRunTo(Fields.armConeStack-100);
    }
    public void liftSmallGoal() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.armBackwardsLow);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderForwardLow);
    }
    public void liftSuperSlightly() {sliderRunTo(Fields.sliderSuperLow);}
    public void openClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
        delay(1);
    }
    public void fastOpenClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.rightClawPickup);
    }
    public void depositClaw() {
        robot.rightClaw.setPosition(Fields.rightClawDeliver);
        robot.leftClaw.setPosition(Fields.rightClawDeliver);
    }

    public void resetLift() {
        armRunTo(Fields.armGround);
        sliderRunTo(Fields.sliderGround);
        delay(1.5);
    }
    public void fastResetLift() {
        armRunTo(Fields.armGround);
        sliderRunTo(Fields.sliderGround);
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

