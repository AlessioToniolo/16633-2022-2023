package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.AutoHelper;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class CascadeRNGAuto extends LinearOpMode {
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

        AutoHelper fun = new AutoHelper(robot, runtime);

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
                .lineToSplineHeading(new Pose2d(61, -54, Math.toRadians(0)))
                .build();
        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineTo(new Vector2d(62, -10))
                .addTemporalMarker(1.2, ()->{})
                .build();

        // Zone trajs
        /*
        Trajectory zone3 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .forward(24.5)
                .build();

        Trajectory zone1 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .back(25)
                .build();
        Trajectory zone2 = drive.trajectoryBuilder(new Pose2d(seven.end().getX(), seven.end().getY(), Math.toRadians(0)))
                .back(4)
                .build();

         */

        waitForStart();
        if (isStopRequested()) return;

        // Start Code
        robot.closeClaw();
        fun.delay(0.5);
        fun.liftSlightly();


    }
}
