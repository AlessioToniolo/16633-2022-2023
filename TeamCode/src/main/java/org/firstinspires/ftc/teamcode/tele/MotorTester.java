package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.BaseRobot;

@Disabled
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
        robot.drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




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
        leftforward = gamepad1.left_stick_y * speed *-1;
        rightforward = gamepad1.right_stick_y * speed*-1;
        if(gamepad1.left_stick_x > 0)//left on the trigger is front right is rear
        {
            // if to the right
            leftrear = gamepad1.left_stick_x * speed;
            robot.drive.leftRear.setPower(leftrear);//0 and3

        }
        else if(gamepad1.left_stick_x < 0 )
        {
            // if to the left
            leftfront = gamepad1.left_stick_x * speed;
            robot.drive.leftFront.setPower(leftfront);

        }
        else if(gamepad1.right_stick_x < 0 )
        {
            // if to the left
            rightfront = gamepad1.right_stick_x * speed;
            robot.drive.rightFront.setPower(rightfront);

        }else if(gamepad1.right_stick_x > 0 )
        {
            // if to the left
            rightrear = gamepad1.right_stick_x * speed;
            robot.drive.rightRear.setPower(rightrear);

        }
        else{
            robot.drive.leftFront.setPower(leftforward);
            robot.drive.rightFront.setPower(rightforward);
            robot.drive.leftRear.setPower(leftforward);
            robot.drive.rightRear.setPower(rightforward);
        }
        // Kinematics
        telemetry.addLine("LeftY:"+gamepad1.left_stick_y);
        telemetry.addLine("LeftX:"+gamepad1.left_stick_x);
        telemetry.addLine("RightY:"+gamepad1.right_stick_y);
        telemetry.addLine("RightX:"+gamepad1.right_stick_x);
        telemetry.update();



    }





}
