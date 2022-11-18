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
public class HoustonAuto extends LinearOpMode {
    // robot with drive
    BaseRobot robot = new BaseRobot();
    // OpenCV
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

        // Go to leftmost square
        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(2, -56))
                .build();
        // Drive near pole on left side of field
        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineTo(new Vector2d(12, -20))
                .build();
        // FIRST DEPOSIT
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineToLinearHeading(new Pose2d(12, -5.5, Math.toRadians(48)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // Back away to middle of middle square
        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineTo(new Vector2d(35, -13.5))
                .build();
        // SECOND PICKUP
        Trajectory five = drive.trajectoryBuilder(four.end())
                .lineToSplineHeading(new Pose2d(58.5, -14.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, ()->{
                    fastOpenClaw();
                    liftConeStack();
                })
                .build();
        // Drive out of pickup and lift cone
        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineTo(new Vector2d(45, -7))
                .addTemporalMarker(0.1, () -> {
                    fastLiftHigh(true, 0.5);
                })/*
                .addTemporalMarker(1, () -> {
                    fastLiftHigher(true, 0.5);
                })*/
                .build();
        // SECOND DEPOSIT
        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineToLinearHeading(new Pose2d(31.5, -2, Math.toRadians(-35)))
                .build();
        // Back away to middle of middle square
        Trajectory eight = drive.trajectoryBuilder(seven.end())
                .lineTo(new Vector2d(33, -8))
                .build();
        // THIRD PICKUP
        Trajectory nine = drive.trajectoryBuilder(eight.end())
                .lineToSplineHeading(new Pose2d(61, -8, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, ()->{
                    fastOpenClaw();
                    liftConeStackLess();
                })
                .build();
        /*
        // TODO not sure what this trajectory does
        Trajectory ten = drive.trajectoryBuilder(nine.end())
                .lineTo(new Vector2d(61, -8))
                .build();
         */
        // I THINK Drive out of pickup to avoid hitting low junction
        Trajectory ten = drive.trajectoryBuilder(nine.end())
                .lineTo(new Vector2d(45, -7))
                .addTemporalMarker(0.1, () -> {
                    fastLiftHigh(true, 0.5);
                })
                .build();
        // Third Deposit
        Trajectory eleven = drive.trajectoryBuilder(ten.end())
                .lineToLinearHeading(new Pose2d(32.5, -0.3, Math.toRadians(-35)))
                .build();

        // Zone trajs
        Trajectory zone1 = drive.trajectoryBuilder(eleven.end())
                .lineToLinearHeading(new Pose2d(2.3, -11.3, Math.toRadians(90)))
                .build();
        Trajectory zone3 = drive.trajectoryBuilder(eleven.end())
                .lineToLinearHeading(new Pose2d(69, -12, Math.toRadians(90)))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Start Code
        robot.closeClaw();
        delay(0.5);
        liftSlightly();

        // OpenCV Code
        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        // Auto Code
        drive.followTrajectory(one);
        drive.followTrajectory(two);
        fastLiftLower(false, 0.55);
        delay(0.2);
        drive.followTrajectory(three);
        openClaw();
        drive.followTrajectory(four);
        drive.followTrajectory(five);
        closeClaw();
        liftSlightly();
        delay(0.2);
        drive.followTrajectory(six);
        drive.followTrajectory(seven);
        openClaw();
        drive.followTrajectory(eight);
        drive.followTrajectory(nine);
        closeClaw();
        liftSlightly();
        delay(0.2);
        drive.followTrajectory(ten);
        drive.followTrajectory(eleven);
        openClaw();


        if (zone == 1) {
            drive.followTrajectory(zone1);
        }
        if (zone == 3) {
            drive.followTrajectory(zone3);
        }
        resetLift();
    }

    // FUNCTIONS
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
            sliderRunTo(Fields.sliderBackwardsHigh-80, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-80, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack+10);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStackLess() {
        sliderRunTo(Fields.sliderConeStack-100);
        armRunTo(Fields.armConeStack+50);
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
    public void fastCloseClaw() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
        delay(.5);
    }

    // Helper functions
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

