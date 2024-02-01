package eu.qrobotics.centerstage.teamcode.cv;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public int detectionSize = 0;
    private int detectionAvgOf = 2;
    public int callCount = 0;

    public double maxUpdateTime=0;
    private double aTagFacingAngle = 0.0;


    private Robot robot;

    private ElapsedTime cycleTime = new ElapsedTime(0);


    public double getTime() {
        return cycleTime.seconds();
    }

    public ATagDetector(Robot robot, HardwareMap hardwareMap, int portalId){
        this.robot = robot;

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        processor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(processor)
                .setLiveViewContainerId(portalId)
                .build();
        cycleTime.reset();
    }

    public void close() {
        visionPortal.close();
        robot.drive.useAprilTagDetector = false;
        detected = false;
    }

    public void detect() {
        callCount++;
        if(callCount == 200){
            callCount = 0;
            maxUpdateTime = 0;
        }
        detections = processor.getDetections();

        if (detections == null ||
            detections.size() == 0) {
            detected = false;
            debugText = "#1: No detections";
            return;
        }

        maxUpdateTime = Math.max(maxUpdateTime, cycleTime.seconds());
        cycleTime.reset();
        detectionSize = detections.size();
        detected = true;

        if (detections.size() == 1) {
            estimatedPose = new Pose2d();
            AprilTagDetection aTag = detections.get(0);
            Vector2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);

            double xRobot = (aTagPose.getX() - aTag.ftcPose.y) * (1.0 / detectionAvgOf);
            double yRobot = (aTagPose.getY() + aTag.ftcPose.x) * (1.0 / detectionAvgOf);
            double hRobot = (aTagFacingAngle - aTag.ftcPose.yaw) * (1.0 / detectionAvgOf);
            estimatedPose = new Pose2d(xRobot, yRobot, hRobot);
            return;
        }

        detections.sort((april1, april2) -> (april1.ftcPose.range < april2.ftcPose.range ? 1 : 0));
        double xRobot = 0, yRobot = 0, hRobot = 0;

        for (int i = 0; i < detectionAvgOf; i++) {
            AprilTagDetection aTag = detections.get(i);
            Vector2d aTagPose = AprilPoses.aprilPoses.get(aTag.id);

            xRobot = xRobot + (aTagPose.getX() - aTag.ftcPose.y) * (1.0 / detectionAvgOf);
            yRobot = yRobot + (aTagPose.getY() + aTag.ftcPose.x) * (1.0 / detectionAvgOf);
            hRobot = hRobot + (aTagFacingAngle - Math.toRadians(aTag.ftcPose.yaw)) * (1.0 / detectionAvgOf);
        }
        estimatedPose = new Pose2d(xRobot, yRobot,hRobot);
        debugText = "#2: Size of " + detections.size();
    }


}
