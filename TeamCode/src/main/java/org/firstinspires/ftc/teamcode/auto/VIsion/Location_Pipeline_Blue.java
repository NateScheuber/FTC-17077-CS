package org.firstinspires.ftc.teamcode.auto.VIsion;




import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Location_Pipeline_Blue extends OpenCvPipeline {
    Telemetry telemetry;
    public Location_Pipeline_Blue(Telemetry t){telemetry = t;}

    static final Point topLeft1 = new Point (60,140);
    static final Point bottomRight1 = new Point (80, 160);

    static final Point topLeft2 = new Point(270, 130);
    static final Point bottomRight2 = new Point(290, 150);
    Mat mat = new Mat();
    Mat region1 = new Mat();
    Mat region2 = new Mat();

    static final Scalar WHITE = new Scalar(255, 255, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(mat, mat, 2);

        Scalar lowCr = new Scalar(170);
        Scalar highCr = new Scalar(230);


        Core.inRange(mat, lowCr, highCr, mat);

        region1 = mat.submat(new Rect(topLeft1, bottomRight1));
        region2 = mat.submat(new Rect(topLeft2, bottomRight2));


        if(Core.mean(region1).val[0]>100){
            Imgproc.rectangle(mat, topLeft1, bottomRight1, GREEN, 1);
            Imgproc.rectangle(mat, topLeft2, bottomRight2, RED,1);
            //Imgproc.cvtColor(input, mat, Imgproc.COLOR_GRAY2RGB);
            telemetry.addData("Position", "1");
        }
        else if(Core.mean(region2).val[0]>100){
            Imgproc.rectangle(mat, topLeft1, bottomRight1, RED, 1);
            Imgproc.rectangle(mat, topLeft2, bottomRight2, GREEN,1);
            //Imgproc.cvtColor(input, mat, Imgproc.COLOR_GRAY2RGB);
            telemetry.addData("Position", "2");
        }
        else {
            Imgproc.rectangle(mat, topLeft1, bottomRight1, RED, 1);
            Imgproc.rectangle(mat, topLeft2, bottomRight2, RED, 1);
            //Imgproc.cvtColor(input, mat, Imgproc.COLOR_GRAY2RGB);
            telemetry.addData("Position", "3");
        }
        telemetry.update();
        return mat;
    }
}
