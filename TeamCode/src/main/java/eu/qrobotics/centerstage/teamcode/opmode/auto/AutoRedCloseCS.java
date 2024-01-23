package eu.qrobotics.centerstage.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

import eu.qrobotics.centerstage.teamcode.cv.TeamPropPipelineRed;
import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryCloseRed;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "01 AutoRedCloseCS")
public class AutoRedCloseCS extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;

    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;
    public static int cycleCount = 2;
    int trajectoryIdx = 0;

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
        robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
        robot.sleep(0.7);
        if (teamProp != 2) {
            robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT;
        }
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.sleep(0.2);
    }

    void placePixel(boolean goToBackboard) {
        // TODO: outtake in score mode
        robot.sleep(0.05);

        // TODO: adaptive path following towards apriltag idfk
        if (goToBackboard) {
            gotobackboard();
        }

        // TODO: place pixelussy and retract outtake
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryCloseRed.START_POSE);
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        teamProp = cameraTeamProp();
        robot.start();
        // TODO: is this order? ^^^^

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        trajectories = TrajectoryCloseRed.getTrajectories(robot, cycleCount, teamProp, false, true);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

        solvePurplePixel();
        if (teamProp == 1) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.LEFT;
        } else if (teamProp == 2) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
        } else if (teamProp == 3) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.RIGHT;
        }

        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(1);
        // we are now kinda in front of the backboard
        placePixel(false);
        robot.drive.followTrajectory(trajectories.get(2));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        trajectoryIdx = 3;
        // TODO: *cica* cycles
        for (int i = 1; i <= cycleCount; i++) {
            robot.intake.dropdownState = Intake.DropdownState.STACK_5;
            robot.sleep(0.5);
            robot.intake.dropdownState = Intake.DropdownState.DOWN;
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.intake.dropdownState = Intake.DropdownState.UP;
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.sleep(0.1);

            placePixel(false);
            if (i == cycleCount) {
                robot.outtake.rotateState = Outtake.RotateState.CENTER;
                robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
                robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
            }
            robot.sleep(0.2);
            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.sleep(0.02);
        }

        robot.sleep(1);

        robot.stop();
    }

}
