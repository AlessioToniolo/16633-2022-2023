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
public class TripleConeAuto extends LinearOpMode {
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
                .lineTo(new Vector2d(3, -56))
                .build();
        // Drive near pole on left side of field
        Trajectory two = drive.trajectoryBuilder(one.end())//speed this guy up
                .lineTo(new Vector2d(12, -20), SampleMecanumDrive.getVelocityConstraint(AutoFields.speedySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(AutoFields.speedyAccel))
                .addTemporalMarker(0.2, ()->{
                    fastLiftLower(false, .6);
                })
                .build();
        // FIRST DEPOSIT
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineToLinearHeading(new Pose2d(8.5, -3.5, Math.toRadians(AutoFields.highFrontAngle)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(AutoFields.speedyAccel))
                .build();
        // PICKUP
        Trajectory four = drive.trajectoryBuilder(new Pose2d(three.end().getX(), three.end().getY(), Math.toRadians(0)))
                .lineTo(new Vector2d(AutoFields.autoConePickup, -7), SampleMecanumDrive.getVelocityConstraint(AutoFields.speedySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, ()->{
                    liftConeStack();
                })
                .build();
        // Drive out of pickup and lift cone
        Trajectory five = drive.trajectoryBuilder(four.end())
                .lineTo(new Vector2d(45, -7))
                .addTemporalMarker(0.1, () -> {
                    fastLiftLower(true, 0.25);
                })
                .build();
        // SECOND DEPOSIT
        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToLinearHeading(new Pose2d(25.7, -2.2, Math.toRadians(AutoFields.highBackAngle)))
                .build();
        Trajectory alternativeFive = drive.trajectoryBuilder(four.end())
                .lineToLinearHeading(new Pose2d(25.7, -2.2, Math.toRadians(AutoFields.highBackAngle)))
                .addTemporalMarker(0.1, () -> {
                    fastLiftLower(true, .5);
                })
                .build();
        Trajectory alternativeSeven = drive.trajectoryBuilder(alternativeFive.end())
                .lineToLinearHeading(new Pose2d(AutoFields.autoConePickup, -7, Math.toRadians(0)))
//                .addTemporalMarker(0.1, () -> {
//                    liftConeStack4();})
                .build();
        Trajectory recenter = drive.trajectoryBuilder(alternativeFive.end())
                .lineTo(new Vector2d(26, -7))
                .addTemporalMarker(0.1, () -> {
                    liftConeStack4();})
                .build();
        /*
        Trajectory sixHalf = drive.trajectoryBuilder(six.end())
                .addTemporalMarker(0, () -> lowerChainBar(0.8, 80))
                .forward(-1.5)
                .build();

         */
        Trajectory seven = drive.trajectoryBuilder(six.end())
                .lineTo(new Vector2d(33, -14))//11
                .build();

        // Zone trajs
        Trajectory zone3 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .forward(24.5)
                .build();

        Trajectory zone1 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .back(25)
                .build();
        Trajectory zone2 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .back(4)
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

        telemetry.addLine(""+zone);
        telemetry.update();

        // Auto Code
        drive.followTrajectory(one);//strafe LEft
        drive.followTrajectory(two);//strafe forward
        delay(0.2);
        drive.followTrajectory(three);//strafe forward a bit more and turn
        openClaw();//drop cone
        drive.turn(Math.toRadians(-48));//turn
        drive.followTrajectory(four);//run to cone stack
        closeClaw();//close claw
        clearConeFromStack();//lift slider a bit
        delay(0.5);
//        drive.followTrajectory(five);
//        drive.followTrajectory(six);
        //drive.followTrajectory(sixHalf);
        drive.followTrajectory(alternativeFive);//deliver from the back
        delay(.5);
        openClaw();
        drive.followTrajectory(recenter);
        drive.followTrajectory(alternativeSeven);
        closeClaw();//close claw
        clearConeFromStack();//lift slider a bit
        delay(0.5);
        drive.followTrajectory(alternativeFive);//deliver from the back
        delay(1);
        openClaw();
        delay(.5);
        resetLift();
        drive.turn(Math.toRadians(35));



        if (zone == 1) {
            drive.followTrajectory(zone1);
            openClaw();
        } else if (zone == 3) {
            drive.followTrajectory(zone3);
            openClaw();
        } else {

            drive.followTrajectory(zone2);
        }
    }

    // FUNCTIONS
    public void clearConeFromStack() {
        sliderRunTo(Fields.sliderForwardMid);
    }

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
            sliderRunTo(Fields.sliderBackwardsHigh-75, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-70, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void fastLiftHigher(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh+50);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh);
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
            sliderRunTo(Fields.sliderBackwardsHigh-150);
            armRunTo(Fields.armBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-200);
            armRunTo(Fields.armForwardHigh);
        }
    }
    public void fastLiftLower(boolean depositBackwards, double power) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderBackwardsHigh-150, power);
            armRunTo(Fields.armBackwardsHigh, power);
        } else {
            sliderRunTo(Fields.sliderForwardHigh-140, power);
            armRunTo(Fields.armForwardHigh, power);
        }
    }
    public void dunk() {
        sliderRunTo(robot.slider.getCurrentPosition()+200);
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack+60);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStack4() {
        sliderRunTo(Fields.coneStack4);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStackLess() {
        sliderRunTo(Fields.sliderConeStack-200);
        armRunTo(Fields.armConeStack);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderForwardLow);
    }
    public void liftSuperSlightly() {sliderRunTo(Fields.sliderSuperLow);}
    public void liftHalfway(double power) {
        sliderRunTo(Fields.sliderBackMid, power);
        armRunTo(Fields.armBackwardsMid-300, power);
    }
    public void openClaw() {
        robot.rightClaw.setPosition(Fields.rightClawPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
    }


    public void resetLift() {
        armRunTo(Fields.armAutoGround, 0.8);
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
    public void lowerChainBar(double power, int difference) {
        armRunTo(Fields.armBackwardsHigh+difference, power);
        //sliderRunTo(Fields.sliderBackwardsHigh+difference);
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
            //dfsdfa
        }
    }
}

