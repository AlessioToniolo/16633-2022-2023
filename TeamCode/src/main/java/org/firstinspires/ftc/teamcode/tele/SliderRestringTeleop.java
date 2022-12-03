package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@TeleOp
@Disabled
public class SliderRestringTeleop extends LinearOpMode {


    BaseRobot robot = new BaseRobot();
    double sliderPower=0;
    double sideSliderPower=0;
    @Override
    public void runOpMode(){

        sliderPower=gamepad2.left_stick_y/2.0;
        sideSliderPower=gamepad2.right_stick_y/2.0;
        robot.sideSlider.setPower(sideSliderPower);
        robot.slider.setPower(sliderPower);
    }
}