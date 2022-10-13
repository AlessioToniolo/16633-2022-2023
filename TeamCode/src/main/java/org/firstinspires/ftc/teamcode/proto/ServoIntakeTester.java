package org.firstinspires.ftc.teamcode.proto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Servo Tester")
public class ServoIntakeTester extends LinearOpMode {
    int armPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        BaseRobot robot = new BaseRobot();
        robot.init(hardwareMap);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            /*
            if (gamepad1.right_trigger > 0) {
                robot.intakeTester1.setPower(-1);
                robot.intakeTester1.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                robot.intakeTester1.setPower(1);
                robot.intakeTester1.setPower(-1);
            }
            */
            // Control Arm with Right and Left Triggers
            double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            // Limit Power to -0.4 to 0.4
            if (armMotorPower > 0.4) {
                armMotorPower = 0.4;
                armPos += gamepad1.right_trigger * 8;
            }

            if (armMotorPower < -0.4) {
                armMotorPower = -0.4;
                armPos -= gamepad1.left_trigger * 3;
            }
            robot.slider1.setTargetPosition(armPos);
            robot.slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider1.setPower(0.7);

            robot.slider2.setTargetPosition(armPos);
            robot.slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider2.setPower(0.7);
        }
    }
}
