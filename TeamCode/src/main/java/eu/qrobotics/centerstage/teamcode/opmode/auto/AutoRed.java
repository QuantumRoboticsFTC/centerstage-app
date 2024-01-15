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

import eu.qrobotics.centerstage.teamcode.cv.TeamPropPipelineRed;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryTestRed;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#00 AutoRed")
public class AutoRed extends LinearOpMode {

    Robot robot;
    List<Trajectory> trajectories;

    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera

    int cameraTeamProp() {
        int readFromCamera = noDetectionFlag;

        OpenCvCamera camera;
        TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();

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

    void gotobackboard() {
        ;
    }

    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(0));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

        // TODO: place pixelussy
        robot.intake.dropdownState = Intake.DropdownState.UP;
        robot.sleep(0.07);
    }

    void placePixel(Outtake.DiffyHortizontalState hState, boolean goToBackboard) {
        // TODO: outtake in score mode
        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.setTargetHeight(Elevator.TargetHeight.FIRST_LINE);
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.outtake.diffyHState = hState;
        robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
        robot.sleep(0.8);

        // TODO: adaptive path following towards apriltag idfk
        if (goToBackboard) {
            gotobackboard();
        }

        // TODO: place pixelussy and retract outtake
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.15);
        robot.drive.setMotorPowers(-0.9, -0.9, -0.9, -0.9);
        robot.sleep(0.1);

        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(0.3);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryTestRed.START_POSE);
        int teamProp = cameraTeamProp();

        robot.start();
        robot.intake.dropdownState = Intake.DropdownState.DOWN;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.sleep(0.05);

        if (teamProp == -10) {
            return;
        }

        trajectories = TrajectoryTestRed.getTrajectories(teamProp);

        solvePurplePixel();
        robot.outtake.clawState = Outtake.ClawState.CLOSED;

        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.05);
        // we are now kinda in front of the backboard
        placePixel(Outtake.DiffyHortizontalState.CENTER, true);

        // TODO: *cica* cycles

        // park
        robot.drive.followTrajectory(trajectories.get(2));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.3);

        robot.stop();
    }

}
