package org.firstinspires.ftc.teamcode.robotParts;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class blueFinder extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat centerCrop;
    Mat rightCrop;
    double leftavgfin;
    double centeravgfin;
    double rightavgfin;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);

    Scalar greenColor = new Scalar(0,255.0,0);

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        org.opencv.core.Rect leftRect = new org.opencv.core.Rect(1,1,213,359);
        org.opencv.core.Rect centerRect = new org.opencv.core.Rect(214,1,213,359);
        org.opencv.core.Rect rightRect = new org.opencv.core.Rect(427,1,213,359);
        input.copyTo(outPut);

        Imgproc.rectangle(outPut, leftRect, rectColor,2);
        Imgproc.rectangle(outPut, centerRect, rectColor, 2);
        Imgproc.rectangle(outPut, rightRect, rectColor,2);

        leftCrop = YCbCr.submat(leftRect);
        centerCrop = YCbCr.submat(centerRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 1);
        Core.extractChannel(rightCrop, rightCrop, 1);
        Core.extractChannel(centerCrop, centerCrop, 1);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar centeravg = Core.mean(centerCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgfin = leftavg.val[0];
        centeravgfin = centeravg.val[0];
        rightavgfin = rightavg.val[0];

        if((leftavgfin<centeravgfin)&&(leftavgfin<rightavgfin)){
            Imgproc.rectangle(outPut, leftRect, greenColor,2);
            Imgproc.rectangle(outPut, rightRect, rectColor,2);
            Imgproc.rectangle(outPut, centerRect, rectColor,2);
        } else if ((centeravgfin<leftavgfin)&&(centeravgfin<rightavgfin)) {
            Imgproc.rectangle(outPut, centerRect, greenColor,2);
            Imgproc.rectangle(outPut, rightRect, rectColor,2);
            Imgproc.rectangle(outPut, leftRect, rectColor,2);
        }else{
            Imgproc.rectangle(outPut, rightRect, greenColor,2);
            Imgproc.rectangle(outPut, centerRect, rectColor,2);
            Imgproc.rectangle(outPut, leftRect, rectColor,2);
        }
        return outPut;
    }
}