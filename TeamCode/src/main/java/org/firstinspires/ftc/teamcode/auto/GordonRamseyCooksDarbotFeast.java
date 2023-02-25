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
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class GordonRamseyCooksDarbotFeast extends LinearOpMode {
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


        // 2/25/2023 changes: increase deposit x by 1, decrease deposit y by 1.5


        // Go to junction (PRELOAD)
        Trajectory one = drive.trajectoryBuilder(startPose).splineTo(new Vector2d(27.5, -3.5), Math.toRadians(120))
                // 1.3
                .addTemporalMarker(.8, () -> {
                    fun.liftFrontHigh(1);
                }).build();
        // Go cone stack #1
        Trajectory two = drive.trajectoryBuilder(one.end()).lineToLinearHeading(new Pose2d(36.5, -9.8, Math.toRadians(0)))
                .addTemporalMarker(0.9, () -> {
                    fun.liftConeStack(0.5);
                    fun.clawOpen();
                }).build();
        Trajectory three = drive.trajectoryBuilder(two.end()).lineToSplineHeading(new Pose2d(60.5, -7, Math.toRadians(0))).build();
        // Deposit #1
        Trajectory four = drive.trajectoryBuilder(three.end(), true)
                .lineToSplineHeading(new Pose2d(34, -8, toRadians(0))).build();
        Trajectory five = drive.trajectoryBuilder(four.end(), true)
                .lineToSplineHeading(new Pose2d(21, -3.5, toRadians(-90))).build();
        //Go cone stack #2
        Trajectory six = drive.trajectoryBuilder(five.end()).lineToLinearHeading(new Pose2d(36.5, -7, Math.toRadians(0)))
                .addTemporalMarker(0.2, () -> {
                    fun.liftConeStack4(0.5);
                })
                .addTemporalMarker(1.75, () -> {
                    fun.clawOpen();
                })
                .build();
        Trajectory seven = drive.trajectoryBuilder(six.end(), false)
                .lineToSplineHeading(new Pose2d(60.5, -7, Math.toRadians(0))).build();
        // Deposit #2
        Trajectory eight = drive.trajectoryBuilder(seven.end(), true)
                .lineToSplineHeading(new Pose2d(34, -8, toRadians(0))).build();
        Trajectory nine = drive.trajectoryBuilder(eight.end(), true)
                .lineToSplineHeading(new Pose2d(20.5, -3, toRadians(-90))).build();
        // Go cone stack #3
        Trajectory ten = drive.trajectoryBuilder(nine.end()).lineToLinearHeading(new Pose2d(36.5, -7, Math.toRadians(0)))
                .addTemporalMarker(0.2, () -> {
                    fun.liftConeStack3(0.5);
                })
                .addTemporalMarker(1.75, () -> {
                    fun.clawOpen();
                })
                .build();
        Trajectory eleven = drive.trajectoryBuilder(ten.end(), false)
                .lineToSplineHeading(new Pose2d(60.5, -7, Math.toRadians(0))).build();
        // Deposit #3
        Trajectory twelve = drive.trajectoryBuilder(eleven.end(), true)
                .lineToSplineHeading(new Pose2d(34, -8, toRadians(0))).build();
        Trajectory thirteen = drive.trajectoryBuilder(twelve.end(), true)
                .lineToSplineHeading(new Pose2d(20, -3.5, toRadians(-90))).build();
        Trajectory goMiddle = drive.trajectoryBuilder(thirteen.end(), false).lineTo(new Vector2d(19, -9)).build();
        // PARKING ZONES
        Trajectory zoneOne = drive.trajectoryBuilder(thirteen.end(), false)
                .addTemporalMarker(1, () -> {
                    fun.resetAll();
                })
                .lineToSplineHeading(new Pose2d(10, -9, Math.toRadians(80))).build();
        Trajectory zoneTwo = drive.trajectoryBuilder(thirteen.end(), false)
                .addTemporalMarker(1, () -> {
                    fun.resetAll();
                })
                .lineToSplineHeading(new Pose2d(34, -12, toRadians(90))).build();
        Trajectory zoneThree = drive.trajectoryBuilder(thirteen.end(), false)
                .addTemporalMarker(1, () -> {
                    fun.resetAll();
                })
                .lineTo(new Vector2d(55.5, -10)).build();

        telemetry.update();
        telemetry.addLine("🏎️🏎️ MODE PUSH, MODE PUSH 🏎️🏎️");
        telemetry.speak("Mode push, Mode push");

        waitForStart();
        if (isStopRequested()) return;
        double zone = ZoneDetectionPipeline.getZone();

        // Raise Arm to deposit position
        fun.hoverForward(0.5);
        // PRELOAD
        drive.followTrajectory(one);
        fun.lowerArmFrontSlightlyFromHigh(115);
        delay(.4);
        fun.clawDeliver();
        delay(0.25);
        fun.clawClose();
        // CONE STACK #1
        drive.followTrajectory(two);
        drive.followTrajectory(three);
        fun.clawClose();
        delay(0.25);
        fun.liftBackHigh(.6, 0.3);
        // DEPOSIT #1
        drive.followTrajectory(four);
        drive.followTrajectory(five);
        fun.lowerArmBackSlightlyFromHigh(-105);
        fun.clawDeliver();
        delay(0.25);
        fun.clawClose();
        // CONE STACK #2
        drive.followTrajectory(six);
        drive.followTrajectory(seven);
        fun.clawClose();
        delay(0.25);
        fun.liftBackHigh(.6, 0.3);
        // DEPOSIT #2
        drive.followTrajectory(eight);
        drive.followTrajectory(nine);
        fun.lowerArmBackSlightlyFromHigh(-105);
        fun.clawDeliver();
        delay(0.25);
        fun.clawClose();
        // CONE STACK #3
        drive.followTrajectory(ten);
        drive.followTrajectory(eleven);
        fun.clawClose();
        delay(0.25);
        fun.liftBackHigh(.6, 0.3);
        // DEPOSIT #3
        drive.followTrajectory(twelve);
        drive.followTrajectory(thirteen);
        fun.lowerArmBackSlightlyFromHigh(-105);
        fun.clawDeliver();
        delay(0.4);
        fun.clawClose();
        drive.followTrajectory(goMiddle);

        // OpenCV Code

        camera.stopStreaming();
        camera.closeCameraDevice();
        telemetry.addLine("" + zone);
        telemetry.update();

        if (zone == 1) {
            // Zone 1
            drive.followTrajectory(zoneOne);
            fun.clawOpen();
        } else if (zone == 3) {
            // Zone 3
            drive.followTrajectory(zoneThree);
            fun.clawOpen();
        } else {
            // Zone 2
            drive.followTrajectory(zoneTwo);
            fun.clawOpen();
        }
        delay(5);
    }

    public void delay(double t) {
        runtime.reset();
        while (runtime.seconds() < t && !isStopRequested()) {
        }
    }
}
