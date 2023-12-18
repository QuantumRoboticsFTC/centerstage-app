package eu.qrobotics.centerstage.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import eu.qrobotics.centerstage.teamcode.cv.TeamPropPipeline;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryTest;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Field Inspection AUTto")
public class AutoFieldInspection extends LinearOpMode {

    Robot robot;

    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera

    int cameraTeamProp() {
        int readFromCamera = noDetectionFlag;

        OpenCvCamera camera;
        TeamPropPipeline teamPropPieline = new TeamPropPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.setPipeline(teamPropPieline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // :salute:
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            readFromCamera = teamPropPieline.getTeamProp();
            telemetry.addData("getTeamProp() ", teamPropPieline.getTeamProp());
            telemetry.addData("getAvg() ", teamPropPieline.getMax());
            telemetry.addData("getCnt() ", teamPropPieline.getCount());
            telemetry.addData("isRed ", teamPropPieline.isPropRed());
            telemetry.addData("readFromCamera ", readFromCamera);
            telemetry.update();
        }

        if (isStopRequested()) {
            return robotStopFlag;
        }

        camera.closeCameraDeviceAsync(() -> {});
        return readFromCamera;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.start();

        int teamProp = cameraTeamProp();

        if (teamProp == -10) {
            return;
        }

        robot.sleep(2.5);

        robot.drive.setMotorPowers(0.7, 0.7, 0.7, 0.7);
        robot.sleep(0.5);
        robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
        robot.sleep(1.5);
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(1);

        robot.stop();
    }

}
