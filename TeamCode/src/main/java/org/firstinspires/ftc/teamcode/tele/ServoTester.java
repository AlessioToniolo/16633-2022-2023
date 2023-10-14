package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;

import java.lang.reflect.Field;

@TeleOp
public class ServoTester extends LinearOpMode {
    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen  = new ColorfulTelemetry(telemetry);
    double rightServoPosition = Fields.rightClawClose;
    double leftServoPosition = Fields.leftClawClose;

    double servoPosition = Fields.v4bIntake;

    double sliderPosition = Fields.sliderIntake;

    double climberPosition;

    //static FINALS
    public static final double speed = .1;




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        climberPosition = robot.climber.getCurrentPosition();
        waitForStart();
        pen.setColor("white").setUnderLine(true);
        while(!isStopRequested() && opModeIsActive()){
            rightServoPosition += gamepad1.left_stick_x*.001;
            leftServoPosition += gamepad1.right_stick_x*.001;
            if(rightServoPosition >1)rightServoPosition=1;
            else if(rightServoPosition<0)rightServoPosition=0;
            if(leftServoPosition > 1)leftServoPosition=1;
            else if(leftServoPosition < 0)leftServoPosition=0;
            robot.leftClaw.setPosition(leftServoPosition);
            robot.rightClaw.setPosition(rightServoPosition);

            servoPosition -= gamepad2.right_stick_y*.01;
            if(servoPosition >1)servoPosition=1;
            else if(servoPosition<0)servoPosition=0;

            if(Math.abs(gamepad2.left_stick_y) > .3){
                sliderPosition -= gamepad2.left_stick_y*2;
            }
            if(sliderPosition < Fields.sliderIntake)sliderPosition = Fields.sliderIntake;

            robot.v4bServo.setPosition(servoPosition);
            robot.sliderRunTo(sliderPosition, Fields.sliderPower);
            if(Math.abs(gamepad2.right_trigger-gamepad2.left_trigger)>.1){
                climberPosition += (gamepad2.right_trigger-gamepad2.left_trigger)*10;
            }
            robot.climberRunTo(climberPosition, 1);

            pen.addLine("gamepad2" + gamepad2.left_stick_y);
            pen.addLine("Right Servo = G1 leftX");
            pen.addLine("Left Servo = G1 rightX");
            pen.addLine("v4b = G2 rightY");
            pen.addLine("Slider = G2 leftY");
            pen.addLine("Arm = G2 Triggers");
            pen.addLine("V4b: " + servoPosition);
            pen.addLine("TArget Slider: " + sliderPosition);
            pen.addLine("CLIMBER Target Pos" + climberPosition);
            pen.addLine("CLIMBER POS" + robot.climber.getCurrentPosition());
            pen.addLine("Slider: " + robot.slider.getCurrentPosition());
            pen.addLine("LEFT ClAW: " + leftServoPosition);
            pen.addLine("RIGHT CLAW: " + rightServoPosition);
            pen.update();
        }
    }




}
