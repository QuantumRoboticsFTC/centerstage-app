package eu.qrobotics.centerstage.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
import eu.qrobotics.centerstage.teamcode.cv.TeamPropDetectionRed;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRAMiddle;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBWall_2_4;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Audience Truss
@Config
@Autonomous(name = "02 AutoRAMiddle // Red Audience Middle", group = "main")
public class AutoRAMiddle extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;
    List<Trajectory> trajectoriesLeft;
    List<Trajectory> trajectoriesCenter;
    List<Trajectory> trajectoriesRight;

    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetectionRed;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;
    public static int cycleCount = 3;

    ElapsedTime autoTimer = new ElapsedTime(100);
    ElapsedTime bigIntakeTimer = new ElapsedTime(100);
    ElapsedTime intakeTimer = new ElapsedTime(100);
    ElapsedTime trajectoryTimer = new ElapsedTime(100);
    double intakeTimerLimit = 0.5;
    double bigIntakeTimerLimit = 1.2;

    private ATagDetector aprilDetector;

    int cameraTeamProp() {
        int readFromCamera = noDetectionFlag;

        teamPropDetectionRed = new TeamPropDetectionRed();

        telemetry.addData("Webcam 1", "Initing");
        telemetry.update();

        visionPortalTeamProp = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(teamPropDetectionRed)
                .enableLiveView(true)
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
            }
            telemetry.addData("Webcam 1", "Ready");
            telemetry.update();
        }

        if (isStopRequested()) {
            robot.stop();
            return robotStopFlag;
        }

        while (!isStarted()) {
            readFromCamera = teamPropDetectionRed.getTeamProp();
            telemetry.addData("Case", readFromCamera);
            telemetry.addData("left", teamPropDetectionRed.leftValue());
            telemetry.addData("cent", teamPropDetectionRed.centreValue());
            telemetry.addData("thresh", teamPropDetectionRed.threshold);
            telemetry.update();
        }

        visionPortalTeamProp.stopStreaming();

        return readFromCamera;
    }

    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(0));
        if (teamProp == 1) {
            // left
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        } else if (teamProp == 3) {
            // right
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        } else {
            // center
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        }
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (0.15 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.25) {
                robot.outtake.clawState = Outtake.ClawState.CLOSED;
                robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            }
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;

        robot.drive.followTrajectory(trajectories.get(1));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
    }

    void placePixel() {
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.2);
    }

    void retryOuttake() {
        robot.drive.followTrajectory(trajectories.get(10));
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                robot.outtake.rotateState = Outtake.RotateState.CENTER;
                robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
            }
            robot.sleep(0.01);
        }
        robot.sleep(0.25);
        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.1);
        robot.drive.followTrajectory(trajectories.get(11));
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                robot.outtake.rotateState = Outtake.RotateState.RIGHT;
                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
            }
            robot.sleep(0.01);
        }
        robot.sleep(0.2);
        placePixel();
        robot.sleep(0.2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryRAMiddle.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
        robot.outtake.clawState = Outtake.ClawState.CLOSED;

        trajectoriesLeft = TrajectoryRAMiddle.getTrajectories(robot, cycleCount, false, 1);
        trajectoriesCenter = TrajectoryRAMiddle.getTrajectories(robot, cycleCount, false, 2);
        trajectoriesRight = TrajectoryRAMiddle.getTrajectories(robot, cycleCount, false, 3);

        teamProp = cameraTeamProp();

        if (teamProp == 1) {
            trajectories = trajectoriesLeft;
        } else if (teamProp == 2) {
            trajectories = trajectoriesCenter;
        } else if (teamProp == 3 || teamProp == -1) {
            trajectories = trajectoriesRight;
        }

        while(!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }

        robot.start();
        autoTimer.reset();

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        solvePurplePixel();
        robot.sleep(0.2);

        // 2 -> go to stack
        robot.drive.followTrajectory(trajectories.get(2));
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (trajectoryTimer.seconds() > 0.2) {
                robot.intake.intakeMode = Intake.IntakeMode.IN;
            }
            robot.sleep(0.01);
        }


        bigIntakeTimer.reset();
        intakeTimer.reset();
        while (robot.intake.pixelCount() < 2 &&
                intakeTimer.seconds() < intakeTimerLimit
                && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("pixel count", robot.intake.pixelCount());
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.intake.dropdownState = robot.intake.dropdownState.previous();
        while (robot.intake.pixelCount() < 2 &&
                intakeTimer.seconds() < intakeTimerLimit
                && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("pixel count", robot.intake.pixelCount());
            telemetry.update();
            robot.sleep(0.01);
        }
        while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit &&
                opModeIsActive() && !isStopRequested()) {
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                telemetry.addData("pixel count", robot.intake.pixelCount());
                telemetry.update();
                robot.sleep(0.01);
            }
            if (robot.intake.pixelCount() == 2) {
                robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
            } else {
                robot.intake.dropdownState = robot.intake.dropdownState.previous();
            }
            telemetry.addData("pixel count", robot.intake.pixelCount());
            telemetry.update();
        }
        Intake.intakeSensorsOn = false;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

        int currTraj = 3;
        // TODO: *cica* cycles
        for (int i = 1; i <= cycleCount; i++) {
            // 3/6/9 <-> go to backdrop
            Intake.intakeSensorsOn = true;
            robot.drive.followTrajectory(trajectories.get(currTraj++));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                }

                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                }

                if (0.6 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.7) {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                }
                if (robot.drive.getPoseEstimate().getX() > 5) {
                    if (i == 1) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    } else if (i == 2) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    }
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                }
                if (robot.drive.getPoseEstimate().getX() > 30 &&
                        robot.intake.pixelCount() == 2) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                }
                robot.sleep(0.01);
            }

            if (robot.intake.pixelCount() != 2) {
                robot.sleep(0.2);
                placePixel();
            }
            while (robot.intake.pixelCount() > 0) {
                retryOuttake();
            }
            Intake.intakeSensorsOn = false;

            if (i == 3) {
                break;
            }

            // 4/7 <-> go to lane
            robot.drive.followTrajectory(trajectories.get(currTraj++));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && robot.drive.getPoseEstimate().getX() >= 45) {
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                }
                if (robot.drive.getPoseEstimate().getX() < 45) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    if (i == 1) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_4;
                    } else if (i == 2) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_2;
                    }
                }
                robot.sleep(0.01);
            }
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;

            // 5/8 <-> go to stack
            robot.drive.followTrajectory(trajectories.get(currTraj++));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.5) {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                }
                robot.sleep(0.01);
            }

            bigIntakeTimer.reset();
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                telemetry.addData("pixel count", robot.intake.pixelCount());
                telemetry.update();
                robot.sleep(0.01);
            }
            robot.intake.dropdownState = robot.intake.dropdownState.previous();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                telemetry.addData("pixel count", robot.intake.pixelCount());
                telemetry.update();
                robot.sleep(0.01);
            }
            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit &&
                    opModeIsActive() && !isStopRequested()) {
                intakeTimer.reset();
                while (robot.intake.pixelCount() < 2 &&
                        intakeTimer.seconds() < intakeTimerLimit
                        && opModeIsActive() && !isStopRequested()) {
                    telemetry.addData("pixel count", robot.intake.pixelCount());
                    telemetry.update();
                    robot.sleep(0.01);
                }
                if (robot.intake.pixelCount() == 2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                } else {
                    robot.intake.dropdownState = robot.intake.dropdownState.previous();
                }
                telemetry.addData("pixel count", robot.intake.pixelCount());
                telemetry.update();
            }
            Intake.intakeSensorsOn = false;
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
        }

        // 10 -> park
        robot.drive.followTrajectory(trajectories.get(12));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.3) {
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                robot.outtake.rotateState = Outtake.RotateState.CENTER;
            }
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(0.5);

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