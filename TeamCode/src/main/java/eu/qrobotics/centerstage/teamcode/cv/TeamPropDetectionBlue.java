package eu.qrobotics.centerstage.teamcode.cv;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class TeamPropDetectionBlue implements VisionProcessor {
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    private Scalar bLowerBound = new Scalar(105.0, 80.0, 35.0);
    private Scalar bUpperBound = new Scalar(120.0, 255.0, 255.0);
//    private Scalar rLowerBound = new Scalar(0.0, 80.0, 35.0);
//    private Scalar rUpperBound = new Scalar(20.0, 255.0, 255.0);

    private int teamProp; // 123 <=> LCR
    private boolean isPropRed; // false = blue, true = red
    private int cnt = 0;
    private double leftVal = 0.0;
    private double centreVal = 0.0;
    public static double threshold = 1e6;

    Mat processMat = new Mat();

    public TeamPropDetectionBlue() {

    }

    public double leftValue() {
        return leftVal;
    }

    public double centreValue() {
        return centreVal;
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

    private void evaluateFrame(boolean isRed, double leftAvg, double centAvg) {
        if (threshold < leftAvg) {
            isPropRed = isRed;
            teamProp = 1;
            leftVal = leftAvg;
            return;
        }
        if (threshold < centAvg) {
            isPropRed = isRed;
            teamProp = 2;
            centreVal = centAvg;
            return;
        }
        isPropRed = isRed;
        teamProp = 3;
        return;
    }

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        ;
    }

    @Override
    public Object processFrame (Mat input, long captureTimeNanos) {
        ++cnt;

        // 1080x1920
        Rect leftRect = new Rect(800, 1, 400, 539);
        Rect centRect = new Rect(800, 540, 400, 539);

        // BLUE
//        Imgproc.cvtColor(input, processMat, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(processMat, bLowerBound, bUpperBound, processMat);
//
//        evaluateFrame(false, Core.sumElems(processMat.submat(leftRect)).val[0],
//                Core.sumElems(processMat.submat(centRect)).val[1]);

        // RED
        Imgproc.cvtColor(input, processMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(processMat, bLowerBound, bUpperBound, processMat);

        evaluateFrame(true, Core.sumElems(processMat.submat(leftRect)).val[0],
                Core.sumElems(processMat.submat(centRect)).val[0]);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

}