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

        // Build Trajectories
        Pose2d startPose = new Pose2d(35, -72+13.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Go to leftmost square
        Trajectory one = drive.trajectoryBuilder(startPose).splineTo(new Vector2d(29, -8.5), Math.toRadians(120)).build();

        waitForStart();
        if (isStopRequested()) return;

        // Start Code

        drive.followTrajectory(one);

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
