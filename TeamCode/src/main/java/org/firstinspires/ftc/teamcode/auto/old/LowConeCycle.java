package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class LowConeCycle extends LinearOpMode {
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
        Pose2d testStart = new Pose2d(52.5, -12, Math.toRadians(0));
        //drive.setPoseEstimate(startPose);
        drive.setPoseEstimate(testStart);
        openClaw();
        sliderRunTo(Fields.coneStack5);
        armRunTo(Fields.armConeStack);
        delay(1);
        closeClaw();
        sliderRunTo(1400);
        delay(1);
        robot.drive.turn(Math.toRadians(Fields.LowCycleTurnAngle));
        delay(1);
        robot.drive.setPoseEstimate(new Pose2d(52.5, -12, Math.toRadians(Fields.LowCycleTurnAngle)));
        robot.drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(Fields.LowCycleMove).build());
        delay(1.5);
        sliderRunTo(Fields.sliderBackLow);
        armRunTo(Fields.armBackwardsLow, Fields.armSpeed);
        delay(2.5);
        openClaw();
        delay(.1);
        sliderRunTo(0);
        armRunTo(0, Fields.armSpeed);
        delay(1);
        robot.drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(Fields.LowCycleMove).build());
        delay(1);
        robot.drive.setPoseEstimate(new Pose2d(52.5, -12, Math.toRadians(Fields.LowCycleTurnAngle)));
        robot.drive.turn(Math.toRadians(0));
        delay(7);

        Trajectory one = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36, -20))
                .build();
        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToLinearHeading(new Pose2d(38, -3.5, Math.toRadians(132)))
                .build();
        /*
        Trajectory three = drive.trajectoryBuilder(two.end())
                .lineToLinearHeading(new Pose2d(36, -10, Math.toRadians(90)))
                .build();

         */
        Trajectory twoHalf = drive.trajectoryBuilder(two.end())
                .lineTo(new Vector2d(33, -12))
                .build();
        Trajectory three = drive.trajectoryBuilder(twoHalf.end())
                .lineToLinearHeading(new Pose2d(60.5, -13, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, ()->{
                    fastOpenClaw();
                    liftConeStack();
                })
                .build();



        // Zone trajs
//        Trajectory zone3 = drive.trajectoryBuilder(six.end())
//                .lineToLinearHeading(new Pose2d(69, -12, Math.toRadians(90)))
//                .build();
//
//        Trajectory zone1 = drive.trajectoryBuilder(six.end())
//                .lineToLinearHeading(new Pose2d(2.3, -11.3, Math.toRadians(90)))
//                .build();


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
        fastLiftHigh(false, 0.5);
        drive.followTrajectory(two);
        openClaw();
        drive.followTrajectory(twoHalf);
        drive.followTrajectory(three);
        closeClaw();
        liftSlightly();
        delay(.5);
        drive.turn(Math.toRadians(60));
        sliderRunTo(Fields.sliderBackLow, Fields.sliderSpeed);
        armRunTo(Fields.armBackwardsLow, Fields.armSpeed);
        delay(2);
        depositClaw();
        delay(1);
        sliderRunTo(Fields.coneStack4, Fields.sliderSpeed);
        armRunTo(Fields.armConeStack, Fields.armSpeed);
        drive.turn(0);
        closeClaw();
        liftSlightly();
        delay(.2);
        drive.turn(Math.toRadians(60));
        sliderRunTo(Fields.sliderBackLow, Fields.sliderSpeed);
        armRunTo(Fields.armBackwardsLow, Fields.armSpeed);

        delay(20);



//        if (zone == 1) {
//            drive.followTrajectory(zone1);
//        }
//        if (zone == 3) {
//            drive.followTrajectory(zone3);
//        }
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


    public void liftOut() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.sliderBackwardsHigh);
    }
    public void liftConeStack() {
        sliderRunTo(Fields.sliderConeStack);
        armRunTo(Fields.armConeStack);
    }
    public void liftConeStackLess() {
        sliderRunTo(Fields.sliderConeStack-50);
        armRunTo(Fields.armConeStack-50);
    }
    public void liftSmallGoal() {
        sliderRunTo(Fields.sliderBackMid);
        armRunTo(Fields.armBackwardsLow);
    }
    public void liftSlightly() {
        sliderRunTo(Fields.sliderForwardHigh-100);
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
        }
    }

}

