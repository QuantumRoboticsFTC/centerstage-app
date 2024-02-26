package eu.qrobotics.centerstage.teamcode.opmode.auto.regionals.blue;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
import eu.qrobotics.centerstage.teamcode.cv.TeamPropDetectionBlue;
import eu.qrobotics.centerstage.teamcode.opmode.auto.regionals.blue.trajectories.TrajectoryBlueFarTruss;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "04 AutoBlueFarTruss", group = "Blue")
public class AutoBlueFarTruss extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectoriesLeft, trajectoriesCenter, trajectoriesRight;
    List<Trajectory> trajectories;
    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionBlue teamPropDetection;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;
    public static int cycleCount = 2;
    int trajectoryIdx = 0;

    private ATagDetector aprilDetector;

    int cameraTeamProp(int portalId) {
        int readFromCamera = noDetectionFlag;

        teamPropDetection = new TeamPropDetectionBlue(true);

        telemetry.addData("Webcam 1", "Initing");
        telemetry.update();

        visionPortalTeamProp = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(teamPropDetection)
                .setLiveViewContainerId(portalId)
                .build();

        telemetry.setMsTransmissionInterval(50);

        if (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 1", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 1", "Waiting");
                telemetry.addData("State", visionPortalTeamProp.getCameraState());
                telemetry.update();
                sleep(50);
                //sleep(20);
            }
            telemetry.addData("Webcam 1", "Ready");
            telemetry.update();
        }

        if (isStopRequested()&&!isStopRequested()) {
            robot.stop();
            return robotStopFlag;
        }

        while(!isStarted()){
            readFromCamera= teamPropDetection.getTeamProp();
            telemetry.addData("Case", readFromCamera);
            telemetry.addData("ID", teamPropDetection.getID());
            telemetry.addData("Max", teamPropDetection.getMax());
            telemetry.update();
        }

        visionPortalTeamProp.close();

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
        robot.sleep(0.3);

        // TODO: place pixelussy
        if (teamProp != 1) {
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        } else {
            robot.intake.intakeMode = Intake.IntakeMode.OUT;
        }
        robot.sleep(0.5);
    }

    void placePixel(boolean goToBackboard) {
        // TODO: adaptive path following towards apriltag idfk
        if (goToBackboard) {
            gotobackboard();
        }

        // TODO: place pixelussy and retract outtake
        robot.sleep(0.27);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.27);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryBlueFarTruss.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;

        trajectoriesLeft = TrajectoryBlueFarTruss.getTrajectories(robot, cycleCount, 1, false);
        trajectoriesCenter = TrajectoryBlueFarTruss.getTrajectories(robot, cycleCount, 2, false);
        trajectoriesRight = TrajectoryBlueFarTruss.getTrajectories(robot, cycleCount, 3, false);

        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//        aprilDetector=new AprilDetector(hardwareMap,portals[0]);
//        setManualExposure(6,250);
//        robot.setTagDetector(aprilDetector);
        teamProp = cameraTeamProp(portals[1]);
//        aprilDetector.track=false;

        robot.start();
        // TODO: is this order? ^^^^

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        if (teamProp == 1) {
            trajectories = trajectoriesLeft;
        } else if (teamProp == 2) {
            trajectories = trajectoriesCenter;
        } else {
            trajectories = trajectoriesRight;
        }

        solvePurplePixel();
        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        trajectoryIdx = 2;
        if (teamProp == 2) {
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0_CENTER;
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        } else if (teamProp == 3) {
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        } else if (teamProp == 1) {
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        }
        // TODO: *cica* cycles
        for (int i = 1; i <= cycleCount; i++) {
//            if (i == 1) {
//                robot.intake.dropdownState = Intake.DropdownState.STACK_5;
//            } else {
//                robot.intake.dropdownState = Intake.DropdownState.STACK_3;
//            }

            if (i > 1) {
                robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                robot.intake.dropdownState = Intake.DropdownState.STACK_4;
                robot.sleep(0.3);

                robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                // DOWN SI IN by now
                robot.sleep(1.0);
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                robot.intake.dropdownState = Intake.DropdownState.UP;

                robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
            }

            robot.sleep(0.3);
            if (i == 1) {
                robot.intake.dropdownState = Intake.DropdownState.STACK_5;
            } else {
                robot.intake.dropdownState = Intake.DropdownState.STACK_3;
            }
            robot.sleep(0.3);

            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            // DOWN SI IN by now
            robot.sleep(1.0);

            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            placePixel(false);
            robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            if (i != cycleCount) {
                robot.drive.followTrajectory(trajectories.get(trajectoryIdx++));
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
            }
        }

//        robot.outtake.rotateState = Outtake.RotateState.CENTER;
//        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
//        robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
//        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//        robot.sleep(1.25);
//        robot.outtake.clawState = Outtake.ClawState.CLOSED;
//        robot.sleep(0.25);
//        robot.outtake.rotateState = Outtake.RotateState.LEFT;
//        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
//        robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
//        robot.outtake.diffyHState = Outtake.DiffyHortizontalState.RIGHT;
//        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
//        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT;
//        robot.sleep(0.65);
//        robot.outtake.clawState = Outtake.ClawState.OPEN;
//        robot.sleep(0.1);

        robot.stop();
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (aprilDetector.visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 2", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 2", "Waiting");
                telemetry.addData("State:", aprilDetector.visionPortal.getCameraState());
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Webcam 2", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = aprilDetector.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = aprilDetector.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}