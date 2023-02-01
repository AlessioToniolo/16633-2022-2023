package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class DarbotterAuto extends LinearOpMode {
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

        // Init
        AutoFunctions fun = new AutoFunctions(hardwareMap);
        fun.clawClose();

        // Build Trajectories
        Pose2d startPose = new Pose2d(35, -72+13.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Go to junction
        Trajectory one = drive.trajectoryBuilder(startPose).splineTo(new Vector2d(26, -4), Math.toRadians(120))
                .addTemporalMarker(0.6, () -> {
                    fun.liftFrontHigh(1, 0.3);
                }).build();

        // Go cone stack
        Trajectory two = drive.trajectoryBuilder(one.end()).lineTo(new Vector2d(36, -10))
                .addTemporalMarker(0.7, () -> {
                    fun.liftConeStack(0.5);
                }).build();
        Trajectory three = drive.trajectoryBuilder(two.end()).lineToSplineHeading(new Pose2d(60, -9, Math.toRadians(0))).build();

        // Deposit # 1
        Trajectory four = drive.trajectoryBuilder(three.end()).lineTo(new Vector2d(36, -8))
                .addTemporalMarker(0.7, () -> {
                }).build();
        Trajectory five = drive.trajectoryBuilder(four.end()).lineToSplineHeading(new Pose2d(24, -2, Math.toRadians(-60))).build();

        // Cone stack # 2
        Trajectory six = drive.trajectoryBuilder(five.end()).lineTo(new Vector2d(36, -10))
                .addTemporalMarker(0.7, () -> {
                    fun.liftConeStack(0.5);
                }).build();
        Trajectory seven = drive.trajectoryBuilder(six.end()).lineToSplineHeading(new Pose2d(60, -9, Math.toRadians(0))).build();

        // Deposit # 2
        Trajectory eight = drive.trajectoryBuilder(seven.end()).lineTo(new Vector2d(36, -8))
                .addTemporalMarker(0.7, () -> {
                }).build();


        telemetry.update();
        telemetry.speak("READY! ??");
        telemetry.addLine("READY! 🤡");

        waitForStart();
        if (isStopRequested()) return;

        // Start Code
        //fun.liftFrontHigh(0.5, 0.35);
        fun.hoverForward();
        drive.followTrajectory(one);
        delay(1);
        fun.lowerArmFrontSlightlyFromHigh(80);
        fun.clawOpen();
        delay(0.5);
        drive.followTrajectory(two);
        drive.followTrajectory(three);
        fun.clawClose();
        delay(0.5);
        fun.liftBackHigh(.8, 0.3);
        drive.followTrajectory(four);
        drive.followTrajectory(five);
        delay(0.5);
        fun.lowerArmBackSlightlyFromHigh(-150);
        fun.clawOpen();
        delay(0.5);
        drive.followTrajectory(six);
        drive.followTrajectory(seven);
        fun.clawClose();
        delay(0.5);
        fun.liftBackHigh(.8, 0.3);
        //drive.followTrajectory(eight);


        // OpenCV Code
        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        telemetry.addLine("" + zone);
        telemetry.update();


        if (zone == 1) {
            // Zone 1
        } else if (zone == 3) {
            // Zone 3
        } else {
            // Zone 2
        }
    }

    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t && !isStopRequested()) {
            //dfsdfa
        }
    }
}
