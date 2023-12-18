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
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#00 AutoTest")
public class AutoTest extends LinearOpMode {

    Robot robot;
    List<Trajectory> trajectories;

    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera

    int cameraTeamProp() {
        int readFromCamera = noDetectionFlag;

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
            telemetry.update();
        }

        if (isStopRequested()) {
            robot.stop();
            return robotStopFlag;
        }

        camera.closeCameraDeviceAsync(() -> {});
        return readFromCamera;
    }

    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(0));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

        // TODO: place pixelussy
        robot.intake.intakeMode = Intake.IntakeMode.OUT;
        robot.sleep(0.25);
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.sleep(0.1);
    }

    void solveYellowPixel() {
        // TODO: outtake in score mode
        robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
        robot.elevator.setTargetHeight(Elevator.TargetHeight.SCORE);
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.sleep(1);

        // TODO: adaptive path following towards apriltag idfk
        // gotobackboard();

        // TODO: place pixelussy and retract outtake
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.35);

        robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
        robot.elevator.setTargetHeight(Elevator.TargetHeight.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

        // TODO: slowly leave
        robot.drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
        robot.sleep(0.5);
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
        solveYellowPixel();

        // TODO: *cica* cycles

        // park
        robot.drive.followTrajectory(trajectories.get(2));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

        robot.stop();
    }

}
