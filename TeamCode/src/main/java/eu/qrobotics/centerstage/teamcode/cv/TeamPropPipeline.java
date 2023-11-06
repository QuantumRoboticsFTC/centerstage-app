package eu.qrobotics.centerstage.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {
    private Mat modifiedInput = new Mat();
    private Mat leftCrop = new Mat();
    private Mat centCrop = new Mat();
    private Mat rightCrop = new Mat();
    private Mat output = new Mat();
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

    private double maxAverage;
    private int teamProp;

    public int getTeamProp() {
        return teamProp;
    }

    @Override
    public Mat processFrame (Mat input) {
        Imgproc.cvtColor(input, modifiedInput, Imgproc.COLOR_RGB2YCrCb);

        // 1920x1080
        Rect leftRect = new Rect(1, 1, 959, 1079);
        Rect centRect = new Rect(640, 1, 959, 1079);
        Rect rightRect = new Rect(1280, 1, 959, 1079);

        leftCrop = modifiedInput.submat(leftRect);
        centCrop = modifiedInput.submat(centRect);
        rightCrop = modifiedInput.submat(rightRect);

        input.copyTo(output);
        for (int coi = 1; coi <= 2; coi++) {
            Imgproc.rectangle(output, leftRect, rectColor, 1);
            Imgproc.rectangle(output, centRect, rectColor, 1);
            Imgproc.rectangle(output, rightRect, rectColor, 1);

            Core.extractChannel(leftCrop, leftCrop, coi);
            Core.extractChannel(centCrop, centCrop, coi);
            Core.extractChannel(rightCrop, rightCrop, coi);

            Scalar leftAvg = Core.mean(leftCrop);
            Scalar centAvg = Core.mean(centCrop);
            Scalar rightAvg = Core.mean(rightCrop);

            if (maxAverage < leftAvg.val[0]) {
                maxAverage = leftAvg.val[0];
                teamProp = 0;
            }
            if (maxAverage < centAvg.val[0]) {
                maxAverage = centAvg.val[0];
                teamProp = 1;
            }
            if (maxAverage < rightAvg.val[0]) {
                maxAverage = rightAvg.val[0];
                teamProp = 2;
            }
        }
        return output;
    }
}
