package eu.qrobotics.centerstage.teamcode.cv;

import android.util.Size;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

public class ATagDetector {
    public AprilTagProcessor processor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detections;
    public Boolean detected = false;
    public String debugText = "#0: Init";
    public Pose2d estimatedPose = new Pose2d();
    public Pose2d secondPose = new Pose2d();
    public double range = 0;
    public double bearing = 0;
    public double yaw = 0;
    private Robot robot;
    private int detectionAvgOf = 2;
    private double aTagFacingAngle = 0.0;

    private double a=0.4;
    private double prevStateX=0;

    public ATagDetector(Robot robot, HardwareMap hardwareMap, int portalId) {
        this.robot = robot;
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(912.08, 912.08, 608.759, 359.543)
                .build();

        processor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 800))
                .addProcessor(processor)
                .setLiveViewContainerId(portalId)
                .build();
    }
    public void close() {
        visionPortal.close();
        if (robot != null)
            robot.drive.useAprilTagDetector = false;
        detected = false;
    }
    public void detect() {
        detected = false;
        debugText = "#1: No detections";

        try{
            detections = (List<AprilTagDetection>) processor.getDetections().clone();
            if (detections == null ||
                    detections.size() == 0) {

                return;
            }

            detected = true;

            double xRobot = 0, yRobot = 0, hRobot = 0;
            int batchSize=Math.min(detectionAvgOf,detections.size());

            detections.sort((april1, april2) -> (april1.ftcPose.range < april2.ftcPose.range ? 1 : 0));

            for (int i = 0; i < batchSize; i++) {
                AprilTagDetection aTag = detections.get(i);
                Pose2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);

                if (aTag == null ||
                        aTag.ftcPose==null) {
                    continue;
                }

                double angle=Math.toRadians(aTag.ftcPose.yaw-aTag.ftcPose.bearing);
                double dx=Math.sin(angle)*aTag.ftcPose.range;
                double dy=Math.cos(angle)*aTag.ftcPose.range;
                xRobot += (aTagPose.getX() - dy) * 1.0;
                yRobot += (aTagPose.getY() + dx) * 1.0;
                hRobot += aTag.ftcPose.yaw * 1.0;
            }

            xRobot/=batchSize;
            yRobot/=batchSize;
            hRobot/=batchSize;

            estimatedPose = new Pose2d(xRobot, yRobot, -hRobot);
            debugText = "#2: Size of " + batchSize;
        }
        catch (Exception e){

        }

//        aTag = detections.get(0);
//        xRobot=aTag.rawPose.x;
//        yRobot=aTag.rawPose.y;
//        secondPose = new Pose2d(xRobot, yRobot, hRobot);
//        range=aTag.ftcPose.range;
//        yaw=aTag.ftcPose.yaw;
//        bearing=aTag.ftcPose.bearing;
//        debugText = "#2: Size of " + detections.size();
    }
}