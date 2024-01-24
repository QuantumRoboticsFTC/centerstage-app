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
import java.util.Vector;

public class AprilDetector {

    public AprilTagProcessor processor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> currentDetections;

    public Boolean track=false;

    public Boolean detected=false;

    public String debugText="";

    public Pose2d pose2d=new Pose2d();


    public AprilDetector(HardwareMap hardwareMap,int portalId){
        processor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).build();
        processor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(processor)
                .setLiveViewContainerId(portalId)
                .build();
    }

    public void detect(){
        currentDetections = processor.getDetections();
        if(currentDetections==null||currentDetections.size()==0){
            detected=false;
            debugText="0";
        }
        else{
            detected=true;
            currentDetections.sort((april1, april2) -> {return april1.ftcPose.range < april1.ftcPose.range?1:0;});
            double x=0;
            double y=0;
            double heading=0;
            for(int i=0;i<Math.min(2,currentDetections.size());i++){
                AprilTagDetection april=currentDetections.get(i);
                Vector2d pose=AprilPoses.aprilPoses.get(april.id);
                x+=pose.getX() - april.ftcPose.y;
                y+=pose.getY() - april.ftcPose.x;
                heading+=april.ftcPose.bearing;
            }
            double av=Math.min(2,currentDetections.size());
            pose2d=new Pose2d(x/av,y/av,heading/av);
            debugText=String.valueOf(currentDetections.size());
        }
    }


}
