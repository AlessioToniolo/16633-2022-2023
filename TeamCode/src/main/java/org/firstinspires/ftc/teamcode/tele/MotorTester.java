package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@TeleOp
public class MotorTester extends OpMode {
    // Use the class created to define a Robot's hardware
    BaseRobot robot = new BaseRobot();


    double speed = 1; //tracks speed of motors
    @Override
    public void init() {
        // Init hardware variables
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status: ", "Robot Ready");
        telemetry.addLine("");
        telemetry.addData("Warning: ", "Servo Moves on Initalization");

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Reset the position of the servo for intaking
        /*robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
        robot.slider.setPower(1);*/


    }

    @Override
    public void loop() {
        // Modifiable variables for current speed calculations
        double rightforward;
        double leftforward;
        double rightrear;
        double rightfront;
        double leftrear;

        double leftfront;
        double turn;



        // Adjusting the input by the speed cap
        leftforward = gamepad1.left_stick_y * speed;
        rightforward = gamepad1.right_stick_y * speed;
        if(gamepad1.left_stick_x > 0)//left on the trigger is front right is rear
        {
            // if to the right
            leftrear = gamepad1.left_stick_x * speed;
            robot.leftRear.setPower(leftrear);//0 and3

        }
        else if(gamepad1.left_stick_x < 0 )
        {
            // if to the left
            leftfront = gamepad1.left_stick_x * speed;
            robot.leftFront.setPower(leftfront);

        }
        else if(gamepad1.right_stick_x < 0 )
        {
            // if to the left
            rightfront = gamepad1.right_stick_x * speed;
            robot.rightFront.setPower(rightfront);

        }else if(gamepad1.right_stick_x > 0 )
        {
            // if to the left
            rightrear = gamepad1.right_stick_x * speed;
            robot.rightRear.setPower(rightrear);

        }
        else{
            robot.leftFront.setPower(leftforward);
            robot.rightFront.setPower(rightforward);
            robot.leftRear.setPower(leftforward);
            robot.rightRear.setPower(rightforward);
        }
        // Kinematics
        telemetry.addLine("LeftY:"+gamepad1.left_stick_y);
        telemetry.addLine("LeftX:"+gamepad1.left_stick_x);
        telemetry.addLine("RightY:"+gamepad1.right_stick_y);
        telemetry.addLine("RightX:"+gamepad1.right_stick_x);
        telemetry.update();



    }





}
