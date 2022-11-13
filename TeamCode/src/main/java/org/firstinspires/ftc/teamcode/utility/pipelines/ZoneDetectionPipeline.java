package org.firstinspires.ftc.teamcode.utility.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;


import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ZoneDetectionPipeline extends OpenCvPipeline {
    //Upeer and lower HSV Values, use HSV Helper to convert
    //Orange
    public Scalar orangeLower = new Scalar(12 , 93, 0);
    public Scalar orangeUpper = new Scalar(20 ,255,255);

    //Yellow
    public Scalar yellowLower = new Scalar(23 , 81, 0);
    public Scalar yellowUpper = new Scalar(36 ,255,255);

    //Pink
    public Scalar pinkLower = new Scalar(156 , 24, 0);
    public Scalar pinkUpper = new Scalar(168 ,255,255);

    //Green
    public Scalar greenLower = new Scalar(63 , 63, 0);
    public Scalar greenUpper = new Scalar(83 ,255,255);
    //Green
    public Scalar blackLower = new Scalar(0 , 0, 0);
    public Scalar blackUpper = new Scalar(255 ,255,10);



    //Will hold black and white result
    public Mat filtered = new Mat();
    //holds subMat
    public Mat detectionArea = new Mat();

    public static int zone = 0;
    Telemetry telemetry = null;

    public Rect subMat;
    //CREATE all Mats as instance variables so you dont forget to call .release(0 on therm)

    public ZoneDetectionPipeline(Telemetry in, double x, double y,double width, double height){
        telemetry = in;
        if(x<0)x=0;
        if(y<0)y=0;
        if(width<0)width=0;
        if(height<0)height=0;
        subMat = new Rect((int)x,(int)y,(int)width, (int)height);
    }


    @Override
    public Mat processFrame(Mat input) {
        if(subMat.x>=input.width())subMat.x=input.width()-1;
        if(subMat.y>=input.height())subMat.y=input.height()-1;
        if(subMat.width+subMat.x>=input.width())subMat.width=input.width()-subMat.x-1;
        if(subMat.height+subMat.y>=input.height())subMat.height=input.height()-subMat.y-1;


        //Search Pink
        double pinkPercent = determinePercent(input, subMat, pinkLower, pinkUpper);
        telemetry.addLine("Pink: " + pinkPercent);

        //Search Yellow
        double yellowPercent = determinePercent(input, subMat, yellowLower, yellowUpper);
        telemetry.addLine("Yellow: " + yellowPercent);

        //Search Green
        double greenPercent = determinePercent(input, subMat, greenLower, greenUpper);
        telemetry.addLine("Green: " + greenPercent);
        if(pinkPercent>yellowPercent && pinkPercent>greenPercent){
            ZoneDetectionPipeline.zone = 2;
        }
        else if(yellowPercent>pinkPercent&&yellowPercent>greenPercent){
            ZoneDetectionPipeline.zone=1;
        }
        else if(greenPercent>pinkPercent&&greenPercent>yellowPercent){
            ZoneDetectionPipeline.zone=3;
        }

        //Search Orange
        //double orangePercent = determinePercent(input, subMat, orangeLower, orangeUpper);
        //telemetry.addLine("Orange: " + orangePercent);

        //Search Black
        //double blackPercent = determinePercent(input, subMat, blackLower, blackUpper);
        //telemetry.addLine("BLack: " + blackPercent);

        telemetry.addLine("Input Dimentions: Width: " +input.width() + "Height: "+input.height() );
        telemetry.addLine("X:" + subMat.x);
        telemetry.addLine("Y"+subMat.y);
        telemetry.addLine("Width: "+subMat.width);
        telemetry.addLine("Height"+subMat.height);
        telemetry.addLine("Zone: "+zone);

        telemetry.update();

        //For Visualization
        Imgproc.rectangle(input, new Point(subMat.x,subMat.y), new Point(subMat.x+subMat.width,
                subMat.y+subMat.height), new Scalar(255, 255, 255), 1);
        return input;

        //return input;
    }
    public double determinePercent(Mat input, Rect searchIn, Scalar lower, Scalar upper){
        //creat mat to hold hsv
        Mat hsv = new Mat();
        //Convert Color to HSV better for diffreintiating colors
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        //Create Mat to hold filtered values
        Mat filtered = new Mat();

        //turns everyhing not in the range of colors to black inRange(inputMat, lowerRange, UpperRange, outputMat)
        Core.inRange(hsv, lower, upper, filtered);

        //create submat to search in
        Mat detectionArea = filtered.submat(searchIn);

        //Sum the elements
        Scalar sum = Core.sumElems(detectionArea);

        //Retreive the amount from Scalar, should be 255*number of white pixel, kind of janky
        //String.valueOf(Scalar) returns "[num,0,0,0]"" so we isolate the num and cast it to a double
        Double num = Double.valueOf(String.valueOf(sum).split(",",0)[0].replace("[",""));

        //Calculate how many pixels in Detection Area are white
        double numPixelsWhite = (num/255.0);

        //total of pixels in the area
        double area = searchIn.width*searchIn.height;

        //divide nums pixels by total pixels
        double percent = round((numPixelsWhite / area), 3);

        //relese Mats so it doesnt leak memory
        detectionArea.release();
        filtered.release();
        hsv.release();
        return percent *100;

    }
    public double round(double in, int place){
        int cast = (int)(in * (Math.pow(10,place)));
        return (double)cast/Math.pow(10,place);
    }
    //Notes
    //Core.bitwise_and(input, input, mask, filtered);
    //Imgproc.cvtColor(mask, mask, Imgproc.COLOR_HSV2RGB);
    //Imgproc.threshold(input, input, 76, 255, Imgproc.THRESH_BINARY);
    public static int getZone(){
        return zone;
    }




}