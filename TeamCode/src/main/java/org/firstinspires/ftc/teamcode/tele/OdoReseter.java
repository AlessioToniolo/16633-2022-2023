package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;

@TeleOp
public class OdoReseter extends LinearOpMode {


    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen;

    CRServo selectedServo;
    String selectedServoString = "RIGHT";
    private boolean prevDRight=false;
    private boolean prevDDown=false;
    private boolean prevDLeft=false;
    private boolean prevY = false;
    private boolean prevA = false;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        pen = new ColorfulTelemetry(telemetry);
        selectedServo = robot.rightOdoServo;

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            telemetry.addLine();
            telemetry.addLine("push LEFT JOYSTICK up to retract selected ODO SERVO");
            telemetry.addLine("push LEFT JOYSTICK down to lower selected ODO SERVO");
            telemetry.addLine();
            telemetry.addLine("Press DPAD LEFT to select LEFT ODO SERVO");
            telemetry.addLine("Press DPAD RIGHT to select RIGHT ODO SERVO");
            telemetry.addLine("Press DPAD DOWN to select MIDDLE ODO SERVO");
            telemetry.addLine();
            telemetry.addLine("Press A to retract All OdoPods ");
            telemetry.addLine("Press Y to lower All OdoPods ");

            telemetry.addLine();
            telemetry.addLine();

            if(gamepad1.dpad_left && gamepad1.dpad_left != prevDLeft){
                selectedServo = robot.leftOdoServo;
                selectedServoString = "LEFT";
            }
            prevDLeft = gamepad1.dpad_left;

            if(gamepad1.dpad_right && gamepad1.dpad_right != prevDRight){
                selectedServo = robot.rightOdoServo;
                selectedServoString = "RIGHT";
            }
            prevDRight = gamepad1.dpad_right;

            if(gamepad1.dpad_down && gamepad1.dpad_down != prevDDown){
                selectedServo = robot.middleOdoServo;
                selectedServoString = "MIDDLE";
            }
            prevDDown = gamepad1.dpad_down;

            telemetry.addLine("SERVO SELECTED: " + selectedServoString);
            if(gamepad1.left_stick_y < 0){selectedServo.setPower(1); telemetry.addLine("STATE: RETRACTING");}
            else if(gamepad1.left_stick_y>0){selectedServo.setPower(-1);telemetry.addLine("STATE: RELEASING");}
            else {selectedServo.setPower(0);telemetry.addLine("STATE: PAUSED");}


            if(gamepad1.a && gamepad1.a != prevA){
                robot.retractOdoPods(telemetry);
            }
            prevA = gamepad1.a;

            if(gamepad1.y && gamepad1.y != prevY){
                robot.lowerOdoPods(telemetry);
            }
            prevY = gamepad1.y;

            telemetry.update();

        }


    }

    private void retractAll(){
        robot.rightOdoServo.setPower(1);
        robot.leftOdoServo.setPower(1);
        robot.middleOdoServo.setPower(1);
    }
    private void lowerAll(){
        robot.rightOdoServo.setPower(-1);
        robot.leftOdoServo.setPower(-1);
        robot.middleOdoServo.setPower(-1);
    }
    private void pauseAll(){
        robot.rightOdoServo.setPower(0);
        robot.leftOdoServo.setPower(0);
        robot.middleOdoServo.setPower(0);
    }
}