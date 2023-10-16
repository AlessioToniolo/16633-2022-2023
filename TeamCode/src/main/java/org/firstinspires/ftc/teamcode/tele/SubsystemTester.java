package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;

@TeleOp
public class SubsystemTester extends LinearOpMode {
    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen  = new ColorfulTelemetry(telemetry);
    double rightServoPosition = Fields.rightClawClose;
    double leftServoPosition = Fields.leftClawClose;

    double servoPosition = Fields.v4bIntake;

    double sliderPosition = Fields.sliderIntake;

    double climberPosition = 0;

    //static FINALS
    public static final double speed = .1;
    private boolean prevY2;
    private boolean prevX2;
    private boolean prevB2;
    private int clawPos;
    private boolean prevA2;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.climberReset();
        robot.v4bIntake();
        robot.closeClaw();
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

            servoPosition -= gamepad2.right_stick_y*.05;
            if(servoPosition >1)servoPosition=1;
            else if(servoPosition<0)servoPosition=0;

            if(Math.abs(gamepad2.left_stick_y) > .3){
                sliderPosition -= gamepad2.left_stick_y*10;
            }
            if(sliderPosition < Fields.sliderIntake)sliderPosition = Fields.sliderIntake;

            robot.v4bServo.setPosition(servoPosition);
            robot.sliderRunTo(sliderPosition, Fields.sliderPower);
            if(Math.abs(gamepad2.right_trigger-gamepad2.left_trigger)>.2){
                climberPosition += (gamepad2.right_trigger-gamepad2.left_trigger)*10;
            }
            robot.climberRunTo(climberPosition, climberPosition);

            if(gamepad2.y && gamepad2.y != prevY2){
                sliderPosition = Fields.sliderIntake;
                servoPosition = Fields.v4bIntake;
                rightServoPosition =Fields.rightClawPickup;
                leftServoPosition=Fields.leftClawPickup;
                clawPos=1;
                robot.goToIntake();
            }
            prevY2 = gamepad2.y;
            if(gamepad2.x && gamepad2.x != prevX2){

                sliderPosition = Fields.sliderOuttake;
                servoPosition = Fields.v4bDeposit;


                robot.goToOuttake();
            }
            prevX2 = gamepad2.x;
            if(gamepad2.b && gamepad2.b != prevB2){
                robot.releaseAirplane();
            }
            prevB2 = gamepad2.b;
            if(gamepad2.a && gamepad2.a != prevA2){
                if(clawPos ==0){clawPos=1;robot.openClaw();rightServoPosition=Fields.rightClawDeliver;leftServoPosition=Fields.leftClawDeliver;}
                else if(clawPos ==1){clawPos=2;robot.intakeClaw();rightServoPosition=Fields.rightClawPickup;leftServoPosition=Fields.leftClawPickup;}
                else if(clawPos ==2){clawPos =0;robot.closeClaw();rightServoPosition=Fields.rightClawClose;leftServoPosition=Fields.leftClawClose;}
            }
            prevA2 = gamepad2.a;

            pen.addLine("gamepad2" + gamepad2.left_stick_y);
            pen.addLine("Right Servo = G1 leftX");
            pen.addLine("Left Servo = G1 rightX");
            pen.addLine("v4b = G2 rightY");
            pen.addLine("Slider = G2 leftY");
            pen.addLine("Arm = G2 Triggers");
            pen.addLine("V4b: " + servoPosition);
            pen.addLine("TArget Slider: " + sliderPosition);
            pen.addLine("CLIMBER Target Pos" + robot.climber1.getTargetPosition());
            pen.addLine("CLIMBER 1POS" + robot.getClimberPos());
            pen.addLine("Slider: " + robot.slider.getCurrentPosition());
            pen.addLine("LEFT ClAW: " + leftServoPosition);
            pen.addLine("RIGHT CLAW: " + rightServoPosition);
            pen.update();
        }
    }




}
