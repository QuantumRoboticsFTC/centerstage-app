package eu.qrobotics.centerstage.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import eu.qrobotics.centerstage.teamcode.cv.TeamPropPipeline;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryTest;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#00 AutoTest")
public class AutoTest extends LinearOpMode {

    Robot robot;
    List<Trajectory> trajectories;

    int cameraTeamProp() {
        int readFromCamera = -1;

        OpenCvCamera camera;
        TeamPropPipeline teamPropPieline = new TeamPropPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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
            telemetry.addData("readFromCamera ", readFromCamera);
        }

        if (isStopRequested()) {
            robot.stop();
            return -10;
        }

        camera.closeCameraDeviceAsync(() -> {});
        return readFromCamera;
    }


    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(0));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        // TODO: place pixelussy
        ;
    }

    void solveYellowPixel() {
        // TODO: place pixelussy
        ;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryTest.START_POSE);
        robot.start();

        int teamProp = cameraTeamProp();

        if (teamProp == -10) {
            return;
        }

        trajectories = TrajectoryTest.getTrajectories(teamProp);

        solvePurplePixel();

        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        // we are now kinda in front of the backboard

        // TODO: adaptive path following towards apriltag idfk
        // gotobackboard();

        solveYellowPixel();

        // TODO: *cica* cycles

        //endregion

        // park
        robot.drive.followTrajectory(trajectories.get(2));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

        robot.stop();
    }

}
