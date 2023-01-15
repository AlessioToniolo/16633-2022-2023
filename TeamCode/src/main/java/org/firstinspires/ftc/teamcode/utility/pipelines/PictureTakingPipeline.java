package org.firstinspires.ftc.teamcode.utility.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;



import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.InputStream;


public class PictureTakingPipeline extends OpenCvPipeline {


    Telemetry telemetry = null;
    private Mat cameraInput = new Mat();
    private byte[] byteArray;
    final MatOfByte buf = new MatOfByte();
    private String name;
    private final String sourceRoot = "org/firstinspires/ftc/teamcode/storage/";




    public PictureTakingPipeline(Telemetry in){
        telemetry = in;
    }
    public PictureTakingPipeline(String name, Telemetry in){
        this.name = name;
        this.telemetry = in;
    }

    @Override
    public Mat processFrame(Mat input) {
        cameraInput = input;
        Imgcodecs.imencode(".jpg", input, buf);
        byteArray = buf.toArray();
        Imgcodecs.imwrite(sourceRoot + name + ".jpg",input);

        return input;
    }


//    public File getImage()  {
//        //Encoding the image
//        MatOfByte matOfByte = new MatOfByte();
//        Imgcodecs.imencode(".jpg", cameraInput, matOfByte);
//        //Storing the encoded Mat in a byte array
//        byte[] byteArray = matOfByte.toArray();
//        //Preparing the Buffered Image
//        InputStream in = new ByteArrayInputStream(byteArray);
//        BitMap bufImage = ImageIO.read(in);
//        return bufImage;
//    }




}