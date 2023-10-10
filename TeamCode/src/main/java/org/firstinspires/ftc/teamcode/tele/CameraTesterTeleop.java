package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.utility.Fields;
import org.firstinspires.ftc.teamcode.utility.pipelines.ZoneDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class CameraTesterTeleop extends LinearOpMode {
    WebcamName webcamName;
    OpenCvCamera camera;
    ZoneDetectionPipeline myPipeline;
    double x = Fields.subRectX;
    double y = Fields.subRectY;
    double width = Fields.subRectWidth;
    double height = Fields.subRectHeight;
    boolean prevA = false;
    ColorfulTelemetry pen;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init Camera

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new ZoneDetectionPipeline(telemetry, x,y,width, height);
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
        FtcDashboard.getInstance().startCameraStream(camera,0);
        pen = new ColorfulTelemetry(telemetry);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            pen.setColor(ColorfulTelemetry.Red);
            pen.setBold(true);
            pen.setItalic(true);
            pen.setUnderLine(true);
            pen.addLine("SPEED DATA");

            pen.update();
            if(gamepad1.left_stick_y >0)y+=.1;
            if(gamepad1.left_stick_y <0)y-=.1;
            if(gamepad1.left_stick_x >0)x+=.1;
            if(gamepad1.left_stick_x <0)x-=.1;
            if(gamepad1.right_stick_y >0)height+=.1;
            if(gamepad1.right_stick_y <0)height-=.1;
            if(gamepad1.right_stick_x >0)width+=.1;
            if(gamepad1.right_stick_x <0)width-=.1;
            myPipeline = new ZoneDetectionPipeline(telemetry,x,y,width,height);
            camera.setPipeline(myPipeline);
            if(gamepad1.a && gamepad1.a != prevA){
                takePicture();
            }
            prevA = gamepad1.a;


        }
        // Close Camera
        camera.stopStreaming();
        camera.closeCameraDevice();

    }
    public void takePicture(){

    }
}
