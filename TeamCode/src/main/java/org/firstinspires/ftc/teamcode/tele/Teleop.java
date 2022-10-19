package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@TeleOp
public class Teleop extends LinearOpMode {

    BaseRobot robot = new BaseRobot();
    //dpad stuff
    boolean prevDUp = false;
    boolean prevDDown = false;
    boolean prevDRight = false;
    boolean prevDLeft = false;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.drive.setPoseEstimate(new Pose2d(12,12));
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            // Read pose
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    ));
            robot.drive.update();//updates the drive if it is follwing a path async


            checkDPads();
        }


    }
    public void checkDPads(){
        if(gamepad1.dpad_up&&gamepad1.dpad_up!=prevDUp){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .forward(24)
                    .build());
            prevDUp= gamepad1.dpad_up;
        }
        if(gamepad1.dpad_down&&gamepad1.dpad_down!=prevDDown){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .back(24)
                    .build());
            prevDDown= gamepad1.dpad_down;
        }
        if(gamepad1.dpad_right&&gamepad1.dpad_right!=prevDRight){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeRight(24)
                    .build());
            prevDRight= gamepad1.dpad_right;
        }
        if(gamepad1.dpad_left&&gamepad1.dpad_left!=prevDLeft){
            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(24)
                    .build());
            prevDLeft= gamepad1.dpad_left;
        }
    }
    public void doTelemetry() {
        telemetry.addLine("Positon:" + robot.drive.getPoseEstimate());

    }
}
