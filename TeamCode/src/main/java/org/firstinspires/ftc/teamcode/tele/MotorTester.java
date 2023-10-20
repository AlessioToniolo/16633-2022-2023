package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.tele.OldTeleop.round;

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
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




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
        if(Math.abs(gamepad2.right_stick_x) > .1 || Math.abs(gamepad2.left_stick_x) > .1 || Math.abs(gamepad2.left_stick_y)>.1){
            double gamepadDegree;
            gamepadDegree = Math.atan2(Math.toRadians(-gamepad2.left_stick_y),Math.toRadians(gamepad2.left_stick_x)); //normal way of doing it


            //x and y are doubles in the range [-1,1] which are just the cos and sin of the angle you want to drive
            double x = round(Math.cos(gamepadDegree))*speed;//find x and y using cos and sin and then multiply them by the speed
            double y = round(Math.sin(gamepadDegree))*speed;
            double leftRearPower = y - x + gamepad2.right_stick_x;
            double leftFrontPower = y + x + gamepad2.right_stick_x;
            double rightRearPower = y + x - gamepad2.right_stick_x;
            double rightFrontPower = y - x - gamepad2.right_stick_x;

            robot.leftFront.setPower(leftFrontPower);
            robot.leftRear.setPower(leftRearPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.rightRear.setPower(rightRearPower);
        }
        if(gamepad1.x){
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Kinematics
        telemetry.addLine("LeftY:"+gamepad1.left_stick_y);
        telemetry.addLine("LeftX:"+gamepad1.left_stick_x);
        telemetry.addLine("RightY:"+gamepad1.right_stick_y);
        telemetry.addLine("RightX:"+gamepad1.right_stick_x);
        telemetry.addLine("rightFront" + robot.rightFront.getCurrentPosition());
        telemetry.addLine("leftFront" + robot.leftFront.getCurrentPosition());
        telemetry.addLine("rightRear" + robot.rightRear.getCurrentPosition());
        telemetry.addLine("leftRear" + robot.leftRear.getCurrentPosition());

        telemetry.update();



    }





}
