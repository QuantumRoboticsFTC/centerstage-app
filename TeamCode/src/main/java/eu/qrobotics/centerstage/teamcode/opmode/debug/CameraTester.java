package eu.qrobotics.centerstage.teamcode.opmode.debug;

import android.widget.ZoomControls;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.concurrent.TimeUnit;
import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@TeleOp(name = "CameraTester", group = "Concept")
public class CameraTester extends LinearOpMode {
    private ATagDetector aprilDetector;
    private int controllerId=0;
    private long exposure=1;
    private int gain=120;
    private int zoom=0;
    private  int whiteBalance=2900;
    private double updateTime=0;
    private StickyGamepad stickyGamepad;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private PtzControl ptzControl;
    private WhiteBalanceControl whiteBalanceControl;
    private ElapsedTime timer;
    private Thread updateCameraThread=new Thread(()->updateCamera());
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera();
        stickyGamepad= new StickyGamepad(gamepad1);
        timer=new ElapsedTime();
        waitForStart();
        updateCameraThread.start();
        while (!isStopRequested()){
            if(stickyGamepad.dpad_down){
                controllerId=(controllerId+1)%4;
            }
            if(stickyGamepad.dpad_up){
                controllerId--;
                if(controllerId<0){
                    controllerId=3;
                }
            }
            telemetry.addData("Detect time ",updateTime);
            telemetry.addData("Detected: ",aprilDetector.detected);
            if (aprilDetector.detected) {
                Pose2d newPose=aprilDetector.estimatedPose;
                telemetry.addData("X ",newPose.getX());
                telemetry.addData("Y ",newPose.getY());
                telemetry.addData("Heading",newPose.getHeading());
                telemetry.addLine();
//                newPose=aprilDetector.secondPose;
//                telemetry.addData("Dist X ",newPose.getX());
//                telemetry.addData("Dist Y ",newPose.getY());
//                telemetry.addLine();
//                telemetry.addData("Yaw ",aprilDetector.yaw);
//                telemetry.addData("Bearing ",aprilDetector.bearing);
//                telemetry.addData("Range ",aprilDetector.range);
            }
            switch (controllerId){
                case 0:
                    if(stickyGamepad.dpad_left){
                        exposure--;
                    }
                    if(stickyGamepad.dpad_right){
                        exposure++;
                    }
                    telemetry.addData("Exposure ",String.valueOf(exposure)+" < ");
                    telemetry.addData("Gain ",String.valueOf(gain)+"");
                    telemetry.addData("Zoom ",String.valueOf(zoom)+"");
                    telemetry.addData("WhiteBalance ",String.valueOf(whiteBalance)+"");
                    break;
                case 1:
                    if(stickyGamepad.dpad_left){
                        gain-=10;
                    }
                    if(stickyGamepad.dpad_right){
                        gain+=10;
                    }
                    telemetry.addData("Exposure ",String.valueOf(exposure)+"");
                    telemetry.addData("Gain ",String.valueOf(gain)+" < ");
                    telemetry.addData("Zoom ",String.valueOf(zoom)+"");
                    telemetry.addData("WhiteBalance ",String.valueOf(whiteBalance)+"");
                    break;
                case 2:
                    if(stickyGamepad.dpad_left){
                        zoom-=10;
                    }
                    if(stickyGamepad.dpad_right){
                        zoom+=10;
                    }
                    telemetry.addData("Exposure ",String.valueOf(exposure)+"");
                    telemetry.addData("Gain ",String.valueOf(gain)+"");
                    telemetry.addData("Zoom ",String.valueOf(zoom)+" < ");
                    telemetry.addData("WhiteBalance ",String.valueOf(whiteBalance)+"");
                    break;
                case 3:
                    if(stickyGamepad.dpad_left){
                        whiteBalance-=100;
                    }
                    if(stickyGamepad.dpad_right){
                        whiteBalance+=100;
                    }
                    telemetry.addData("Exposure ",String.valueOf(exposure)+"");
                    telemetry.addData("Gain ",String.valueOf(gain)+"");
                    telemetry.addData("Zoom ",String.valueOf(zoom)+"");
                    telemetry.addData("WhiteBalance ",String.valueOf(whiteBalance)+" < ");
                    break;
            }
            telemetry.addLine("");
            telemetry.addData("Current Zoom ",ptzControl.getZoom());
            telemetry.addData("Min Zoom ",ptzControl.getMinZoom());
            telemetry.addData("Max Zoom ",ptzControl.getMaxZoom());
            telemetry.addData("Min WhiteBalance ",whiteBalanceControl.getMinWhiteBalanceTemperature());
            telemetry.addData("Max WhiteBalance ",whiteBalanceControl.getMaxWhiteBalanceTemperature());
            telemetry.addLine("");
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            sleep(20);
            gainControl.setGain(gain);
            sleep(20);
            ptzControl.setZoom(zoom);
            sleep(20);
            whiteBalanceControl=aprilDetector.visionPortal.getCameraControl(WhiteBalanceControl.class);
            whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
            sleep(20);
            telemetry.update();
            stickyGamepad.update();
        }
    }
    public void initCamera() {
        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        aprilDetector=new ATagDetector(null,hardwareMap,portals[0]);
        if (aprilDetector.visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 2", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 2", "Waiting");
                telemetry.addData("State", aprilDetector.visionPortal.getCameraState());
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Webcam 2", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            exposureControl = aprilDetector.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            sleep(20);
            gainControl = aprilDetector.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            ptzControl = aprilDetector.visionPortal.getCameraControl(PtzControl.class);
            ptzControl.setZoom(zoom);
            sleep(20);
            whiteBalanceControl=aprilDetector.visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
                sleep(50);
            }
            whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
            sleep(20);
        }
    }
    void updateCamera(){
        timer.startTime();
        while(!isStopRequested()){
            aprilDetector.detect();
            updateTime=timer.time();
            timer.reset();
        }
    }
}