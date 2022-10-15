package org.firstinspires.ftc.teamcode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;

public class SliderTest extends LinearOpMode{
    BaseRobot robot;

    @Override
    public void runOpMode(){
        robot = new BaseRobot();
        robot.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightSlider.setTargetPosition(200);
        robot.leftSlider.setTargetPosition(200);
        robot.rightSlider.setPower(1);
        robot.leftSlider.setPower(1);
    }


}
