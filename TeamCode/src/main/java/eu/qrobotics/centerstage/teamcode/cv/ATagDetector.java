package eu.qrobotics.centerstage.teamcode.cv;

import android.util.Size;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        detections = processor.getDetections();
        if (detections == null ||
                detections.size() == 0) {
            detected = false;
            debugText = "#1: No detections";
            return;
        }
        detected = true;
        double xRobot = 0, yRobot = 0, hRobot = 0;
        if (detections.size() == 1) {
            AprilTagDetection aTag = detections.get(0);
            Pose2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);
            if (aTagPose.getHeading() == Math.PI) {
                xRobot = (aTagPose.getX() + aTag.ftcPose.y) * 1.0;
                yRobot = (aTagPose.getY() - aTag.ftcPose.x) * 1.0;
            } else {
                xRobot = (aTagPose.getX() - aTag.ftcPose.y) * 1.0;
                yRobot = (aTagPose.getY() + aTag.ftcPose.x) * 1.0;
            }
            hRobot = (aTagPose.getHeading() - aTag.ftcPose.yaw) * 1.0;
            estimatedPose = new Pose2d(xRobot, yRobot, hRobot);
        } else {
            detections.sort((april1, april2) -> (april1.ftcPose.range < april2.ftcPose.range ? 1 : 0));
            for (int i = 0; i < detectionAvgOf; i++) {
                AprilTagDetection aTag = detections.get(i);
                Pose2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);
                if (aTagPose.getHeading() == Math.PI) {
                    xRobot = xRobot + (aTagPose.getX() + aTag.ftcPose.y) * (1.0 / detectionAvgOf);
                    yRobot = yRobot + (aTagPose.getY() - aTag.ftcPose.x) * (1.0 / detectionAvgOf);
                } else {
                    xRobot = xRobot + (aTagPose.getX() - aTag.ftcPose.y) * (1.0 / detectionAvgOf);
                    yRobot = yRobot + (aTagPose.getY() + aTag.ftcPose.x) * (1.0 / detectionAvgOf);
                }
                hRobot = hRobot + (aTagPose.getHeading() - aTag.ftcPose.yaw) * (1.0 / detectionAvgOf);
            }
            estimatedPose = new Pose2d(xRobot, yRobot, hRobot);
        }
        xRobot = 0;
        yRobot = 0;
        hRobot = 0;
        AprilTagDetection aTag = detections.get(0);
        Pose2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);
        if (aTagPose.getHeading() == Math.PI) {
            xRobot = (aTagPose.getX() + aTag.ftcPose.y) * 1.0;
            yRobot = (aTagPose.getY() - aTag.ftcPose.x) * 1.0;
        } else {
            xRobot = (aTagPose.getX() - aTag.ftcPose.y) * 1.0;
            yRobot = (aTagPose.getY() + aTag.ftcPose.x) * 1.0;
        }
        hRobot = (aTagPose.getHeading() - aTag.ftcPose.yaw) * 1.0;
        xRobot+=0.3035 * hRobot;
        yRobot-=0.7127* hRobot;
        secondPose = new Pose2d(xRobot, yRobot, hRobot);
        range=aTag.ftcPose.range;
        yaw=aTag.ftcPose.yaw;
        bearing=aTag.ftcPose.bearing;
        debugText = "#2: Size of " + detections.size();
    }
}