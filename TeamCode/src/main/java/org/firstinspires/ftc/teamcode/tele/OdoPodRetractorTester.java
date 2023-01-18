package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;

@TeleOp
public class OdoPodRetractorTester extends LinearOpMode {


    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen;
    boolean hasCalled = false;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        pen = new ColorfulTelemetry(telemetry);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            if(!hasCalled) {
                robot.retractOdoPods(telemetry);
                hasCalled = true;
            }


        }
    }
}