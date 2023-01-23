package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;

@TeleOp
public class SliderRestringTeleop extends LinearOpMode {


    BaseRobot robot = new BaseRobot();
    ColorfulTelemetry pen;
    double sliderPower=0;
    double sideSliderPower=0;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
         pen = new ColorfulTelemetry(telemetry);
         waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            pen.addLine("Push both Joysticks up to reset slides and tighten the strings");

            sliderPower = gamepad2.left_stick_y / 2.0;
            sideSliderPower = gamepad2.right_stick_y / 2.0;
            robot.sideSlider.setPower(sideSliderPower);
            robot.slider.setPower(sliderPower);
            pen.addLine("Slider Power " + sliderPower);
            pen.addLine("SideSlider Power " + sideSliderPower);
            pen.update();
        }
    }
}