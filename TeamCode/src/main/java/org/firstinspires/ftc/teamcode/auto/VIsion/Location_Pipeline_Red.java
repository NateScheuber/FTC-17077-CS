package org.firstinspires.ftc.teamcode.auto.VIsion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Location_Pipeline_Red extends OpenCvPipeline {

    Telemetry telemetry;

    /*
    public Signal_Pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

     */

    public int position = 1;


    static final Point topLeft1 = new Point (160,140);
    static final Point bottomRight1 = new Point (180, 160);

    static final Point topleft2 = new Point(450, 140);
    static final Point bottomRight2 = new Point(460, 140);
    Mat HSV = new Mat();
    public enum Location{
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location;

    static final Scalar WHITE = new Scalar(255, 255, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);


    static final Rect LEFT_ROI = new Rect(topLeft1, bottomRight1);
    static final Rect MIDDLE_ROI = new Rect(topleft2, bottomRight2);

    static double PERCENT_COLOR_THRESHOLD = 0.5;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,50, 100);
        Scalar highHSV = new Scalar(20,100,255);

        Core.inRange(HSV, lowHSV, highHSV, HSV);
        Mat left = HSV.submat(LEFT_ROI);
        Mat middle = HSV.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0]/LEFT_ROI.area()/255;
        double middleValue = Core.sumElems(middle).val[0]/MIDDLE_ROI.area()/255;

        left.release();
        middle.release();

        telemetry.addData("Left Raw Data", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle Raw Data", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left Percentage", Math.round(leftValue*100)+"%");
        telemetry.addData("Middle Percentage", Math.round(middleValue*100)+"%");

        boolean positionLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean positionMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if(positionLeft){
            location = Location.LEFT;
            telemetry.addData("Prop Location", "left");
        }
        else if(positionMiddle){
            location = Location.MIDDLE;
            telemetry.addData("Prop Location", "middle");
        }
        else{
            location = Location.RIGHT;
            telemetry.addData("Prop Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(HSV, HSV, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(HSV, LEFT_ROI, location == Location.LEFT? GREEN:WHITE);
        Imgproc.rectangle(HSV, MIDDLE_ROI, location == Location.MIDDLE? GREEN:WHITE);

        return HSV;
    }
    public Location getLocation(){
        return location;
    }

}
