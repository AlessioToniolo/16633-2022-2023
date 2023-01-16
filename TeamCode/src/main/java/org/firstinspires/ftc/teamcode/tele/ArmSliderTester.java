package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;

@TeleOp
public class ArmSliderTester extends LinearOpMode {
    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen;
    int sliderPos = 0;
    int armPos = 0;

    //static FINALS
    public static final double speed = .1;




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        pen = new ColorfulTelemetry(telemetry);
        waitForStart();
        pen.setColor("black").setUnderLine(true);
        while(!isStopRequested() && opModeIsActive()){
            sliderPos += -gamepad1.left_stick_y*10;
            armPos += -gamepad1.right_stick_y*10;

            if(sliderPos >Fields.sliderMaximumTarget)sliderPos=Fields.sliderMaximumTarget;
            else if(sliderPos<0)sliderPos=0;


            if(armPos > Fields.armMaximumTarget)armPos=Fields.armMaximumTarget;
            else if(armPos < 0)armPos=0;


            sliderRunTo(sliderPos);
            armRunTo(armPos);

            pen.addLine("Slider Target: " + sliderPos);
            pen.addLine("Slider Speed: " + (Fields.sliderSpeed/1.0)*100 + "%");
            pen.addLine("ArmTarget: " + armPos);
            pen.addLine("Arm Speed: " + (Fields.armSpeed/1.0)*100 + "%");

            pen.update();
        }

    }

    public void sliderRunTo(int position){
        sliderRunTo(position, Fields.sliderSpeed);
    }
    public void armRunTo(int position){
        armRunTo(position, Fields.armSpeed);
    }

    public void armRunTo(int position, double power){
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);
    }
    public void sliderRunTo(int position, double power){
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(power);
        robot.sideSlider.setTargetPosition(position);
        robot.sideSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sideSlider.setPower(power);
    }


}
