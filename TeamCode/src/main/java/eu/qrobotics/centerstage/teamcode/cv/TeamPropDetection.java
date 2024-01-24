package eu.qrobotics.centerstage.teamcode.cv;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamPropDetection implements VisionProcessor {
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    private Scalar bLowerBound = new Scalar(105.0, 80.0, 35.0);
    private Scalar bUpperBound = new Scalar(120.0, 255.0, 255.0);
    private Scalar rLowerBound = new Scalar(0.0, 80.0, 35.0);
    private Scalar rUpperBound = new Scalar(20.0, 255.0, 255.0);

    private double maxAverage;
    private int teamProp; // 123 <=> LCR
    private boolean isPropRed; // false = blue, true = red
    private int cnt = 0;

    Mat processMat = new Mat();

    public TeamPropDetection(boolean isPropRed){
        this.isPropRed=isPropRed;
    }

    public double getMax() {
        return 0;
//        return maxAverage;
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
        double maxAverage = 0;
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
    public void init(int width, int height, CameraCalibration calib) {
        ;
    }

    @Override
    public Object processFrame (Mat input, long captureTimeNanos) {
        ++cnt;

        // 1920x1080
        Rect leftRect = new Rect(1, 500, 639, 220);
        Rect centRect = new Rect(640, 500, 639, 220);
        Rect rightRect = new Rect(1280, 500, 639, 220);

        // BLUE
        Imgproc.cvtColor(input, processMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(processMat, bLowerBound, bUpperBound, processMat);

        evaluateFrame(false, Core.sumElems(processMat.submat(leftRect)).val[0],
                Core.sumElems(processMat.submat(centRect)).val[1],
                Core.sumElems(processMat.submat(rightRect)).val[2]);

        // RED
        Imgproc.cvtColor(input, processMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(processMat, rLowerBound, rUpperBound, processMat);

        evaluateFrame(true, Core.sumElems(processMat.submat(leftRect)).val[0],
                Core.sumElems(processMat.submat(centRect)).val[1],
                Core.sumElems(processMat.submat(rightRect)).val[2]);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

}
