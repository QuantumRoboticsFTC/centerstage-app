package eu.qrobotics.centerstage.teamcode.cv;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TeamPropDetectionRed extends OpenCvPipeline {
    private Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
    private Scalar rectColor2 = new Scalar(0.0, 255.0, 0.0);
    private Scalar rectColor3 = new Scalar(0.0, 0.0, 255.0);
    private Scalar bLowerBound = new Scalar(105.0, 80.0, 35.0);
    private Scalar bUpperBound = new Scalar(120.0, 255.0, 255.0);
    private Scalar rLowerBound = new Scalar(0.0, 80.0, 35.0);
    private Scalar rUpperBound = new Scalar(20.0, 255.0, 255.0);

    public static int x1 = 1;
    public static int w1 = 359;
    public static int x2 = 360;
    public static int w2 = 359;
    public static int x3 = 720;
    public static int w3 = 359;

    private double maxAverage;
    private int teamProp; // 123 <=> LCR
    private boolean isPropRed; // false = blue, true = red
    private int cnt = 0;

    public double getMax() {
//        return 0;
        return maxAverage;
    }

    public int getCount() {
        return cnt;
    }

    public int getTeamProp() {
        return teamProp;
    }

    public boolean isPropRed() {
        return isPropRed;
    }

    private void evaluateFrame(boolean isRed, double leftAvg, double centAvg, double rightAvg) {
        maxAverage = 0;
        if (maxAverage < leftAvg) {
            maxAverage = leftAvg;
            isPropRed = isRed;
            teamProp = 1;
        }
        if (maxAverage < centAvg) {
            maxAverage = centAvg;
            isPropRed = isRed;
            teamProp = 2;
        }
        if (maxAverage < rightAvg) {
            maxAverage = rightAvg;
            isPropRed = isRed;
            teamProp = 3;
        }
        return;
    }

    @Override
    public Mat processFrame (Mat input) {
        Mat HSV = input;
        Mat AuxiliaryMask = new Mat();
        Mat Blue = new Mat();
        Mat Red = new Mat();
        Mat leftCrop = new Mat();
        Mat centCrop = new Mat();
        Mat rightCrop = new Mat();
        ++cnt;

        // 1080x1920
        Rect leftRect = new Rect(x1, 780, w1, 500);
        Rect centRect = new Rect(x2, 780, w2, 500);
        Rect rightRect = new Rect(x3, 780, w3, 500);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        // RED
        Core.inRange(HSV, rLowerBound, rUpperBound, AuxiliaryMask);
        Core.bitwise_and(HSV, HSV, Red, AuxiliaryMask);

        leftCrop = Red.submat(leftRect);
        centCrop = Red.submat(centRect);
        rightCrop = Red.submat(rightRect);

        Imgproc.rectangle(Red, leftRect, rectColor1, 5);
        Imgproc.rectangle(Red, centRect, rectColor2, 5);
        Imgproc.rectangle(Red, rightRect, rectColor3, 5);

        evaluateFrame(true, Core.mean(leftCrop).val[0],
                Core.mean(centCrop).val[0],
                Core.mean(rightCrop).val[0]);

//        input.release();
//        HSV.release(); // release?
        AuxiliaryMask.release();
//        Red.release();
//        Blue.release();
        leftCrop.release();
        centCrop.release();
        rightCrop.release();
        return Red;
    }
}
