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

@Autonomous
public class TestAuto extends LinearOpMode {
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
        robot.closeClaw();

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

        telemetry.addLine("ready");
        waitForStart();

        if (isStopRequested()) return;

        robot.delay(1);
        double zone = ZoneDetectionPipeline.getZone();
        robot.delay(1);
        camera.stopStreaming();
        camera.closeCameraDevice();

        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(70, -57))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(65, -10))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(135)))
                .build();

        // Zone trajs
        Trajectory zone1 = robot.drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(12, -12))
                .build();

        Trajectory zone2 = robot.drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(60, -12))
                .build();

        drive.followTrajectory(traj1);
        telemetry.addLine("finish");
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        liftHighGoal(false);
        resetLift();
        drive.turn(Math.toRadians(-45));

        if (zone == 1) {
            robot.drive.followTrajectory(zone1);
        } else if (zone == 3) {
            robot.drive.followTrajectory(zone1);
        }

        telemetry.clear();
        telemetry.speak("Free O O", "en-TT", "ALB");
        telemetry.addLine("🤡🤡🤡 LETS GO BRAZY DRIPP MONSTERS!!!!😛😛😛🤙");
    }
    // Auto robot functions
    public void liftHighGoal(boolean depositBackwards) {
        if(depositBackwards){
            sliderRunTo(Fields.sliderHighJunctionLevel);
            armRunTo(Fields.armDepostBackwardsHigh);
        } else {
            sliderRunTo(Fields.sliderHighJunctionLevel);
            armRunTo(Fields.armDepostForwardsHigh);
        }
        delay(3);
        robot.rightClaw.setPosition(Fields.rightClawDeliver);
        robot.leftClaw.setPosition(Fields.leftClawDeliver);
        delay(1);
    }
    public void resetLift() {
        armRunTo(Fields.armPickup);
        sliderRunTo(Fields.sliderGroundPickup);
        robot.leftClaw.setPosition(Fields.leftClawPickup);
        robot.rightClaw.setPosition(Fields.rightClawPickup);
    }
    public void closeClaw() {
        robot.rightClaw.setPosition(Fields.rightClawClose);
        robot.leftClaw.setPosition(Fields.leftClawClose);
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

