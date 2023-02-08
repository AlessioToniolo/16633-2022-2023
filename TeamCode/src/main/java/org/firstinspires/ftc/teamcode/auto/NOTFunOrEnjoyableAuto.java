package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class NOTFunOrEnjoyableAuto extends LinearOpMode {
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

        // Go to junction (PRELOAD)
        Trajectory one = drive.trajectoryBuilder(startPose).splineTo(new Vector2d(27, -2), Math.toRadians(120))
                .addTemporalMarker(1.3, () -> {
                    fun.liftFrontHigh(1);
                }).build();

        // Go cone stack #1
        Trajectory two = drive.trajectoryBuilder(one.end()).lineToLinearHeading(new Pose2d(36.5, -9.8, Math.toRadians(0)))
                .addTemporalMarker(0.9, () -> {
                    fun.liftConeStack(0.5);
                    fun.clawOpen();
                }).build();
        Trajectory three = drive.trajectoryBuilder(two.end()).lineToSplineHeading(new Pose2d(59, -7, Math.toRadians(0))).build();



        telemetry.update();
        telemetry.speak("NBA YOUNGBOY");
        telemetry.addLine("READY! 🤡");

        waitForStart();
        if (isStopRequested()) return;

        // Start Code
        //Raise Arm to deposit position
        fun.hoverForward(0.5);
        //Go to first deposit position
        drive.followTrajectory(one);
        fun.lowerArmFrontSlightlyFromHigh(80);
        //Open Claw for preload
        fun.clawOpen();
        delay(0.25);
        //Go to cone stack #1
        drive.followTrajectory(two);
        drive.followTrajectory(three);
        //Pick Up Cone Stack #1
        fun.clawClose();
        delay(0.25);
        //Set lift to Position for deposit #2
        

        // OpenCV Code
        double zone = ZoneDetectionPipeline.getZone();
        camera.stopStreaming();
        camera.closeCameraDevice();

        telemetry.addLine("" + zone);
        telemetry.update();


        fun.resetAll();
        /*
        if (zone == 1) {
            // Zone 1
            drive.followTrajectory(eleven);
            drive.turn(toRadians(180));
        } else if (zone == 3) {
            // Zone 3
            drive.followTrajectory(thirteen);
        } else {
            // Zone 2
            drive.followTrajectory(twelve);
        }*/
    }

    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t && !isStopRequested()) {
        }
    }
}
