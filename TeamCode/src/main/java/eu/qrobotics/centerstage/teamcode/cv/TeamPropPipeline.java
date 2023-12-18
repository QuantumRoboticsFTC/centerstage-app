package eu.qrobotics.centerstage.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    private Scalar bLowerBound = new Scalar(105.0, 80.0, 35.0);
    private Scalar bUpperBound = new Scalar(120.0, 255.0, 255.0);
    private Scalar rLowerBound = new Scalar(0.0, 80.0, 35.0);
    private Scalar rUpperBound = new Scalar(20.0, 255.0, 255.0);

    private double maxAverage;
    private int teamProp; // 123 <=> LCR
    private boolean isPropRed; // false = blue, true = red
    private int cnt = 0;

    public double getMax() {
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

        // 1920x1080
        Rect leftRect = new Rect(1, 1, 639, 779);
        Rect centRect = new Rect(640, 1, 639, 779);
        Rect rightRect = new Rect(1280, 1, 639, 779);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        // BLUE
        Core.inRange(HSV, bLowerBound, bUpperBound, AuxiliaryMask);
        Core.bitwise_and(HSV, HSV, Blue, AuxiliaryMask);

        leftCrop = Blue.submat(leftRect);
        centCrop = Blue.submat(centRect);
        rightCrop = Blue.submat(rightRect);

        Imgproc.rectangle(Blue, leftRect, rectColor, 1);
        Imgproc.rectangle(Blue, centRect, rectColor, 1);
        Imgproc.rectangle(Blue, rightRect, rectColor, 1);

        evaluateFrame(false, Core.mean(leftCrop).val[0],
                Core.mean(centCrop).val[0],
                Core.mean(rightCrop).val[0]);

        // RED
        Core.inRange(HSV, rLowerBound, rUpperBound, AuxiliaryMask);
        Core.bitwise_and(HSV, HSV, Red, AuxiliaryMask);

        leftCrop = Red.submat(leftRect);
        centCrop = Red.submat(centRect);
        rightCrop = Red.submat(rightRect);

        Imgproc.rectangle(Red, leftRect, rectColor, 1);
        Imgproc.rectangle(Red, centRect, rectColor, 1);
        Imgproc.rectangle(Red, rightRect, rectColor, 1);

        evaluateFrame(true, Core.mean(leftCrop).val[0],
                Core.mean(centCrop).val[0],
                Core.mean(rightCrop).val[0]);

//        input.release();
//        HSV.release();
        AuxiliaryMask.release();
//        Red.release();
//        Blue.release();
        leftCrop.release();
        centCrop.release();
        rightCrop.release();
        return Red;
    }
}
