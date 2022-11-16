package org.firstinspires.ftc.teamcode.auto;
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
public class ZoneAuto extends LinearOpMode{
    BaseRobot robot = new BaseRobot();
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        webcamName = hardwareMap.get(WebcamName .class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new ZoneDetectionPipeline(telemetry, Fields.subRectX,Fields.subRectY,Fields.subRectWidth,Fields.subRectHeight);
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
        waitForStart();
        delay(1);
        double zone = ZoneDetectionPipeline.getZone();
        delay(1);
        camera.stopStreaming();
        camera.closeCameraDevice();

        // Build Trajectories
        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);



        // Zone trajs
        Trajectory zone2 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36, -12))
                .build();
        Trajectory zone3 = drive.trajectoryBuilder(zone2.end())
                .lineToLinearHeading(new Pose2d(69, -12, Math.toRadians(90)))
                .build();

        Trajectory zone1 = drive.trajectoryBuilder(zone2.end())
                .lineToLinearHeading(new Pose2d(2.3, -12, Math.toRadians(90)))
                .build();

        drive.followTrajectory(zone2);
        if(zone==3)drive.followTrajectory(zone3);
        else if(zone ==1)drive.followTrajectory(zone1);



    }
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
        }
    }
}
