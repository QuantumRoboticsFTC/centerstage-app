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
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBMiddle_4n2;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBMiddle_Full5;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Backboard Centerstage
@Config
@Autonomous(name = "01 AutoRBMiddle 4n2 // Red Backdrop Middle", group = "Red")
public class AutoRBMiddle_4n2 extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;

    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetection;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = 2; // TODO: atentie e -1 defapt dra na n avem camera
    public static int cycleCount = 3;
    int trajectoryIdx = 0;
    public static double MAX_DISTANCE = -100;

    ElapsedTime autoTimer = new ElapsedTime(10);
    ElapsedTime bigIntakeTimer = new ElapsedTime(10);
    ElapsedTime intakeTimer = new ElapsedTime(10);
    ElapsedTime trajectoryTimer = new ElapsedTime(10);
    double timerLimit1 = 28;
    double timerLimit2 = 28;
    double intakeTimerLimit = 0.5;
    double bigIntakeTimerLimit = 1.2;

    private ATagDetector aprilDetector;

    void solveTimerLimit2() {
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.intake.dropdownState = Intake.DropdownState.UP;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.clawState = Outtake.ClawState.OPEN;

        // 16 -> go directly to park (from lane)
        robot.drive.followTrajectory(trajectories.get(16));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

    }

    int cameraTeamProp(int portalId) {
        int readFromCamera = noDetectionFlag;

        teamPropDetection = new TeamPropDetectionRed(true);

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
            readFromCamera = teamPropDetection.getTeamProp();
            telemetry.addData("Case", readFromCamera);
            telemetry.addData("ID", teamPropDetection.getID());
            telemetry.addData("Max", teamPropDetection.getMax());
            telemetry.update();
        }

        visionPortalTeamProp.close();

        return readFromCamera;
    }

    void solvePurplePixel() {
        if (teamProp == 1) {
            // left
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
            robot.drive.followTrajectory(trajectories.get(0));
        } else if (teamProp == 3) {
            // right
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
            robot.drive.followTrajectory(trajectories.get(2));
        } else {
            // center
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
            robot.drive.followTrajectory(trajectories.get(1));
        }
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        robot.sleep(0.25);
    }

    void placePixel() {
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.15);
    }

    void retractOuttake() {
        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(0.7);
        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.35);
        robot.outtake.rotateState = Outtake.RotateState.RIGHT;
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
        robot.sleep(0.6);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryRBMiddle_4n2.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

        trajectories = TrajectoryRBMiddle_4n2.getTrajectories(robot, cycleCount, false);

        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//        aprilDetector=new AprilDetector(hardwareMap,portals[0]);
//        setManualExposure(6,250);
//        robot.setTagDetector(aprilDetector);
//        teamProp = cameraTeamProp(portals[1]);
        teamProp = 2;
        while(!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }

//        aprilDetector.track=false;

        robot.start();
        autoTimer.reset();

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        solvePurplePixel();
//        robot.sleep(0.2);
        robot.sleep(0.2);

        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
        if (teamProp != 2) {
            robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
        }
        robot.sleep(0.05);

        // 3 -> go to initial backdrop
        trajectoryTimer.reset();
        robot.drive.followTrajectory(trajectories.get(3));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
            robot.sleep(0.01);
            if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.25) {
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            }
        }
        robot.sleep(0.1);
        placePixel();
        robot.sleep(0.2);

        // TODO: *cica* cycles
        for (int i = 1; i <= cycleCount; i++) {
            if (i == 1) {
                // 4 -> initial go to lane
                robot.drive.followTrajectory(trajectories.get(4));
            } else {
                // 7 -> go to lane after drop
                robot.drive.followTrajectory(trajectories.get(8));
            }

            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    if (i == 1 || i == 3) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                    } else if (i == 2) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_3;
                    }
                }
                robot.sleep(0.01);
            }
            robot.sleep(0.1);

            if (i == 3) {
                // se duce la al 2lea stack
                break;
            }

            // 5 -> go to stack
            robot.drive.followTrajectory(trajectories.get(5));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (robot.drive.getPoseEstimate().getX() < -10) {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                }
                robot.sleep(0.01);
            }

            bigIntakeTimer.reset();
            int currentTargetPixel = 1;
            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                intakeTimer.reset();
                while (robot.intake.pixelCount() < currentTargetPixel &&
                        intakeTimer.seconds() < intakeTimerLimit
                        && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                robot.intake.dropdownState = robot.intake.dropdownState.previous();
                if (currentTargetPixel == 1) currentTargetPixel = 2;
            }

            // 6 -> go to lane in front of drop
            robot.drive.followTrajectory(trajectories.get(6));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                }

                if (0.25 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.35) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                }

                if (0.4 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.5) {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                }
                robot.sleep(0.01);
            }
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
            robot.sleep(0.4);
            robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
            robot.sleep(0.2);

            // 7 -> go to backdrop
            robot.drive.followTrajectory(trajectories.get(7));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
                if (robot.drive.getPoseEstimate().getX() > 5) {
                    if (i == 1) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    } else if (i == 2) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    }
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                }

                robot.sleep(0.01);
            }
//            robot.sleep(0.3);
            robot.sleep(0.1);
            placePixel();
            boolean retry = false;
            while (retry) {
                retractOuttake();
                placePixel();
                retry = false;
            }
        }
        if (cycleCount == 3) {
            // 11 -> go to lane before stack
            robot.drive.followTrajectory(trajectories.get(11));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.sleep(0.1);

            // 12 -> go to stack
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.drive.followTrajectory(trajectories.get(12));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.sleep(0.1);

            bigIntakeTimer.reset();
            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                intakeTimer.reset();
                while (intakeTimer.seconds() < intakeTimerLimit
                        && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                robot.intake.dropdownState = robot.intake.dropdownState.previous();
            }
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

            // 13 -> go to lane after stack
            robot.drive.followTrajectory(trajectories.get(13));
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.3) {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                }
                robot.sleep(0.01);
            }
            robot.sleep(0.1);
            if (timerLimit2 < autoTimer.seconds()) {
                solveTimerLimit2();
            } else {
                // 14 -> go to backdrop (from lane)
                robot.drive.followTrajectory(trajectories.get(14));
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
                    if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    }

                    if (robot.drive.getPoseEstimate().getX() > 5) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                    }
                    robot.sleep(0.01);
                }
                robot.sleep(0.1);
                placePixel();
            }
        }
        robot.intake.dropdownState = Intake.DropdownState.UP;
        robot.sleep(0.15);

        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.clawState = Outtake.ClawState.OPEN;

        robot.sleep(1.25);

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
