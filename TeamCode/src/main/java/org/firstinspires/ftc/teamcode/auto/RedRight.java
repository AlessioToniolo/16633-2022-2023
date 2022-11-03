package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedRight extends LinearOpMode {
    // robot with drive
    BaseRobot robot = new BaseRobot();
    //opencv
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init robot
        robot.init(hardwareMap);
        robot.closeClaw();

        telemetry.addLine("Robot finished initializing");

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

        telemetry.clear();
        telemetry.addLine("Camera finished initializing");

        // Compiling Roadrunner trajectories here
        Pose2d startPose = new Pose2d(36, -60, 0);
        robot.drive.setPoseEstimate(startPose);

        Trajectory traj1 = robot.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(36, -55), Math.toRadians(0))
                .lineTo(new Vector2d(60, -55))
                .build();

        Trajectory traj2 = robot.drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(57, -12))
                .build();

        Trajectory traj3 = robot.drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(135)))
                .build();

        // Zone trajs
        Trajectory zone1 = robot.drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(12, -12))
                .build();

        Trajectory zone2 = robot.drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(60, -12))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        robot.delay(1);
        double zone = ZoneDetectionPipeline.getZone();
        robot.delay(1);
        camera.stopStreaming();
        camera.closeCameraDevice();

        // Run Trajectories
        robot.drive.followTrajectory(traj1);
        telemetry.addLine("finish");
        robot.drive.followTrajectory(traj2);
        robot.drive.followTrajectory(traj3);
        //robot.liftHighGoal(false);
        //robot.resetLift();
        robot.drive.turn(Math.toRadians(-45));

        if (zone == 1) {
            robot.drive.followTrajectory(zone1);
        } else if (zone == 3) {
            robot.drive.followTrajectory(zone1);
        }

        telemetry.clear();
        telemetry.speak("Free O O", "en-TT", "ALB");
        telemetry.addLine("🤡🤡🤡 LETS GO BRAZY DRIPP MONSTERS!!!!😛😛😛🤙");
    }
}
