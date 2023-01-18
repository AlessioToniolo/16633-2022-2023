package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;

@TeleOp
public class ServoTester extends LinearOpMode {
    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen  = new ColorfulTelemetry(telemetry);
    double rightServoPosition = Fields.rightClawClose;
    double leftServoPosition = Fields.leftClawClose;

    //static FINALS
    public static final double speed = .1;




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        pen.setColor("black").setUnderLine(true);
        while(!isStopRequested() && opModeIsActive()){
            rightServoPosition += gamepad1.left_stick_x*.001;
            leftServoPosition += gamepad1.right_stick_x*.001;
            if(rightServoPosition >1)rightServoPosition=1;
            else if(rightServoPosition<0)rightServoPosition=0;
            if(leftServoPosition > 1)leftServoPosition=1;
            else if(leftServoPosition < 0)leftServoPosition=0;
            robot.leftClaw.setPosition(leftServoPosition);
            robot.rightClaw.setPosition(rightServoPosition);


            pen.addLine("LEFT ClAW: " + leftServoPosition);
            pen.addLine("RIGHT CLAW: " + rightServoPosition);
            pen.update();
        }
    }




}
