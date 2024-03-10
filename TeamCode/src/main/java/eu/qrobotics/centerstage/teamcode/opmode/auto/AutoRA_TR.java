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
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRA_TR;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Audience Truss
@Config
@Autonomous(name = "04 AutoRATR // Red Audience Truss", group = "Red")
public class AutoRA_TR extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;

    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetection;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;
    public static int cycleCount = 2;
    int trajectoryIdx = 0;

    ElapsedTime intakeTimer = new ElapsedTime(0);
    ElapsedTime trajectoryTimer = new ElapsedTime(0);
    double intakeTimerLimit = 0.45;

    private ATagDetector aprilDetector;

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
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);

        // TODO: place pixelussy
//        robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.outtake.manualFourbarPos = Outtake.FOURBAR_TRANSFER_POS;
//        robot.sleep(0.5);
        if (teamProp != 2) {
            robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
        }
//        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
    }

    void placePixel() {
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.2);
    }

    void retractOuttake() {
        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(1.25);
        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.35);
        robot.outtake.rotateState = Outtake.RotateState.LEFT;
        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
        robot.sleep(0.65);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryRA_TR.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;

        trajectories = TrajectoryRA_TR.getTrajectories(robot, cycleCount, false);

        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//        aprilDetector=new AprilDetector(hardwareMap,portals[0]);
//        setManualExposure(6,250);
//        robot.setTagDetector(aprilDetector);

        //teamProp = cameraTeamProp(portals[1]);
        teamProp=1;
        waitForStart();

//        aprilDetector.track=false;

        robot.start();

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        solvePurplePixel();
        robot.sleep(0.2);

        // 3 - go to stack
        robot.drive.followTrajectory(trajectories.get(3));
        trajectoryTimer.reset();
        /*
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (trajectoryTimer.seconds() > 0.2) {
                robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                robot.intake.intakeMode = Intake.IntakeMode.IN;
            }
            robot.sleep(0.01);
        }
        */
        robot.sleep(0.1);

        // TODO: *cica* cycles
        for (int i = 1; i <= cycleCount; i++) {
            if (i > 1) {
                // 6 -> go to lane
                robot.drive.followTrajectory(trajectories.get(8));
                trajectoryTimer.reset();
                /*
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    if (trajectoryTimer.seconds() > 0.2) {
                        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                        robot.elevator.elevatorState = Elevator.ElevatorState.TRANSFER;
                    }
                    robot.sleep(0.01);
                }
                */
                robot.sleep(0.1);

                // 7 -> go to stack
                robot.drive.followTrajectory(trajectories.get(9));
                /*
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    if (robot.drive.getPoseEstimate().getX() < -15 &&
                            -20 < robot.drive.getPoseEstimate().getX()) {
                        if (i == 1) {
                            robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                        } else {
                            robot.intake.dropdownState = Intake.DropdownState.STACK_3;
                        }
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    }
                    robot.sleep(0.01);
                }
                */
                intakeTimer.reset();
                while (robot.intake.pixelCount() < 1 && intakeTimer.seconds() < intakeTimerLimit) {
                    robot.sleep(0.1);
                }

                /*
                if (i == 1) {
                    robot.intake.dropdownState = Intake.DropdownState.STACK_4;
                } else {
                    robot.intake.dropdownState = Intake.DropdownState.STACK_2;
                }

                 */
            }
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 && intakeTimer.seconds() < intakeTimerLimit) {
                robot.sleep(0.1);
            }
            /*
            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
            */

            // 4 -> go to lane
            robot.drive.followTrajectory(trajectories.get(6));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            // 5 -> go to backdrop
            robot.drive.followTrajectory(trajectories.get(7));
            trajectoryTimer.reset();
            /*
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (trajectoryTimer.seconds() > 0.15) {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                }
                if (trajectoryTimer.seconds() > 0.2) {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                }

                if (trajectoryTimer.seconds() > 0.35) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                }

                if (robot.drive.getPoseEstimate().getX() > 5) {
                    if (i == 1) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    } else if (i == 2) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    }
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                }

                robot.sleep(0.01);
            }
            robot.sleep(0.3);
            placePixel();
            boolean retry = false;
            while (retry) {
                retractOuttake();
                placePixel();
                retry = false;
            }

             */
        }

        // 8 -> park
        robot.drive.followTrajectory(trajectories.get(10));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(1);

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