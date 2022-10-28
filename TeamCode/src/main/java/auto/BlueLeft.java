package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueLeft extends LinearOpMode {
    BaseRobot robot = new BaseRobot();
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setReverse();
        webcamName = hardwareMap.get(WebcamName .class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new ZoneDetectionPipeline(telemetry, Fields.subRectX,Fields.subRectY,Fields.subRectWidth,Fields.subRectHeight);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(myPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        waitForStart();
        delay(1);
        double zone = ZoneDetectionPipeline.getZone();
        telemetry.addLine("Zone: "+zone);
        delay(1);
        camera.stopStreaming();
        camera.closeCameraDevice();
        robot.strafeInches(1,-24,5);
        telemetry.addLine("hello");
        delay(.5);
        robot.driveStraightInches(1,56,5);
        delay(.5);
        robot.strafeInches(1,36,5);
        delay(.5);
        robot.pointTurnDegrees(1,180,5);
        delay(.5);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setTargetPosition(Fields.sliderHighJunctionLevel);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setTargetPosition(Fields.armDepositForward);
        robot.arm.setPower(1);
        robot.slider.setPower(1);
        delay(2);
        robot.leftClaw.setPosition(Fields.leftClawDeliver);
        robot.rightClaw.setPosition(Fields.rightClawDeliver);
        delay(1);
        robot.leftClaw.setPosition(Fields.rightClawPickup);
        robot.rightClaw.setPosition(Fields.leftClawPickup);
//set claw to open a;jl;oiewqoia;lkjdsa;lkjfdsa
//set claw to close ;LASJFDKA;LJSDFKJJDSAF
        robot.arm.setTargetPosition(0);
        robot.slider.setTargetPosition(0);
        robot.arm.setPower(1);
        robot.slider.setPower(1);
        if(zone==1)
        {
            robot.strafeInches(1,-24,5);
        }
        else if(zone==3)
        {
            robot.strafeInches(1,24,5);
        }
    }
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
        }
    }
}
