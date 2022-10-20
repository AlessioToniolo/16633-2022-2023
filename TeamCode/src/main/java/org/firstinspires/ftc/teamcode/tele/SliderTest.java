package org.firstinspires.ftc.teamcode.tele;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@TeleOp
public class SliderTest extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException{
        BaseRobot robot = new BaseRobot();
        robot.init(hardwareMap);
        boolean prevA = false;
        waitForStart();
        robot.rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!isStopRequested() && opModeIsActive()){

            if(gamepad1.right_trigger>0)robot.rightSlider.setPower(.2);
            else robot.rightSlider.setPower(0);
             if(gamepad1.left_trigger >0)robot.leftSlider.setPower(.2);
             else robot.leftSlider.setPower(0);
            if(gamepad1.a && gamepad1.a != prevA)
                robot.dualSlider.runTo(0);

                prevA = gamepad1.a;
            }
            telemetry.addLine("Right Slider: "+robot.rightSlider.getCurrentPosition());
            telemetry.addLine("Left Slider: "+robot.leftSlider.getCurrentPosition());
            telemetry.addLine("Right Trigger: "+gamepad1.right_trigger);
            telemetry.addLine("Left Trigger: "+gamepad1.left_trigger);
            telemetry.update();


        }

    }



