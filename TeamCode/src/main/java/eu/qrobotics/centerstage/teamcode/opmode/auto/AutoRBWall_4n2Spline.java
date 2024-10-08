package eu.qrobotics.centerstage.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
import eu.qrobotics.centerstage.teamcode.cv.TeamPropDetectionRed;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBMiddle_4n2;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBWall_4n2;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBWall_4n2Spline;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Backboard Centerstage
@Config
@Autonomous(name = "00 AutoRBWall 2+6 // Red Backdrop Wall Spline", group = "main")
public class AutoRBWall_4n2Spline extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;
    List<Trajectory> trajectoriesLeft;
    List<Trajectory> trajectoriesCenter;
    List<Trajectory> trajectoriesRight;

    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetectionRed;
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
    double intakeTimerLimit = 0.3;
    double bigIntakeTimerLimit = 1.2;

    private ATagDetector aTagDetector;
    private Thread updateDetectorThread = new Thread(() -> updateDetector());
    private void updateDetector(){
        while (!isStopRequested()) {
            aTagDetector.detect();
            telemetry.addData("ATag X", aTagDetector.estimatedPose.getX());
            telemetry.addData("ATag Y", aTagDetector.estimatedPose.getY());
            telemetry.addData("ATag Heading", aTagDetector.estimatedPose.getHeading());
            telemetry.update();
        }
    }

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

    void startLineFollower() {
        // go right until we see line
        double directionpwr = 0.5;
        double turnpwr = 0.5;
        robot.drive.updateSensors();
        robot.drive.setMotorPowers(directionpwr, -directionpwr, directionpwr, -directionpwr);
        while (!robot.drive.lineRight() && opModeIsActive() && !isStopRequested()) {
            robot.drive.updateSensors();
            robot.sleep(0.01);
        }
        robot.drive.setMotorPowers(0, 0, 0, 0);

        robot.intake.dropdownState = robot.intake.dropdownState.previous();
        bigIntakeTimer.reset();
        while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit
                && opModeIsActive() && !isStopRequested()) {
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                robot.drive.updateSensors();
                if (robot.drive.lineLeft()) {
                    robot.drive.setMotorPowers(-directionpwr + turnpwr, -directionpwr - turnpwr, -directionpwr - turnpwr, -directionpwr + turnpwr);
                } else if (robot.drive.lineRight()) {
                    robot.drive.setMotorPowers(-directionpwr - turnpwr, -directionpwr + turnpwr, -directionpwr + turnpwr, -directionpwr - turnpwr);
                }
                robot.sleep(0.01);
            }
            if (robot.intake.pixelCount() != 2) {
                robot.intake.dropdownState = robot.intake.dropdownState.previous();
            }
        }
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
    }


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

        if (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 1", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortalTeamProp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
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

    public void configureDetector(int exposureMS, int gain) {
        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        aTagDetector = new ATagDetector(robot, hardwareMap,portals[0]);
        // Wait for the camera to be open, then use the controls
        if (aTagDetector.visionPortal == null) {
            aTagDetector=null;
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (aTagDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 2", "Waiting");
            telemetry.update();
            while ((aTagDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 2", "Waiting");
                telemetry.addData("State:", aTagDetector.visionPortal.getCameraState());
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Webcam 2", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = aTagDetector.visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = aTagDetector.visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
        //updateDetectorThread.start();
    }

    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(0));
        if (teamProp == 1) {
            // left
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        } else if (teamProp == 3) {
            // right
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        } else {
            // center
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        }
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
        robot.sleep(0.15);
    }

    void placePixel() {
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.075);
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
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
        robot.sleep(0.6);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoryRBWall_4n2Spline.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
        robot.outtake.clawState = Outtake.ClawState.CLOSED;

        trajectoriesLeft = TrajectoryRBWall_4n2Spline.getTrajectories(robot, cycleCount, false, 1);
        trajectoriesCenter = TrajectoryRBWall_4n2Spline.getTrajectories(robot, cycleCount, false, 2);
        trajectoriesRight = TrajectoryRBWall_4n2Spline.getTrajectories(robot, cycleCount, false, 3);

//        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
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

//        aprilDetector.track=false;

        robot.start();
        autoTimer.reset();

        if (teamProp == -10) {
            robot.stop();
            return;
        }

        robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
        solvePurplePixel();
        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
        robot.sleep(0.2);

        // 1 -> go to initial backdrop
        trajectoryTimer.reset();
        robot.drive.followTrajectory(trajectories.get(1));
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
        for (int i = 1; i <= Math.min(cycleCount, 2); i++) {
            if (i == 1) {
                // 2 initial -> go to stack <-> normal
                robot.drive.followTrajectory(trajectories.get(2));
            } else if (i == 2) {
                // 4 -> go to stack <-> 2nd time, normal
                robot.drive.followTrajectory(trajectories.get(4));
            }

            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && robot.drive.getPoseEstimate().getX() >= 45) {
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                }
                if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    if (i == 1) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                    } else if (i == 2) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_3;
                    }
                }
                if (robot.drive.getPoseEstimate().getX() <= -33) {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                }
                robot.sleep(0.01);
            }

            Intake.intakeSensorsOn = true;
            bigIntakeTimer.reset();
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.intake.dropdownState = robot.intake.dropdownState.previous();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit
                    && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit &&
                    opModeIsActive() && !isStopRequested()) {
                intakeTimer.reset();
                while (robot.intake.pixelCount() < 2 &&
                        intakeTimer.seconds() < intakeTimerLimit
                        && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                if (robot.intake.pixelCount() == 2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                } else {
                    robot.intake.dropdownState = robot.intake.dropdownState.previous();
                }
            }
            Intake.intakeSensorsOn = false;
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

            if (i == 1) {
                // 3 -> go to backdrop <-> initial
                robot.drive.followTrajectory(trajectories.get(3));
            } else if (i == 2) {
                // 5 -> go to backdrop <-> 2nd time, normal
                robot.drive.followTrajectory(trajectories.get(5));
            }
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                }

                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                }

                if (0.6 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.8) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                }

                if (robot.drive.getPoseEstimate().getX() > 9) {
                    if (i == 1) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    } else if (i == 2) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    }
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                }
                robot.sleep(0.01);
            }
            placePixel();
        }
        if (cycleCount == 3) {
            robot.sleep(0.1);
//            3rd cycle

            // 6 -> go to lane
            robot.drive.followTrajectory(trajectories.get(6));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.2 < trajectoryTimer.seconds() && robot.drive.getPoseEstimate().getX() >= 43) {
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                }
                if (robot.drive.getPoseEstimate().getX() < 43) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                }
                robot.sleep(0.01);
            }
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;

            // 7 -> go to lane before stack
            robot.drive.followTrajectory(trajectories.get(7));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                }
                robot.sleep(0.01);
            }

            // 8 -> go to stack
            robot.drive.followTrajectory(trajectories.get(8));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                }
                robot.sleep(0.01);
            }

            // TODO: u are at stack; intake; leave and backdrop and park
            Intake.intakeSensorsOn = true;
            bigIntakeTimer.reset();
            intakeTimer.reset();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit / 2.0
                    && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            robot.intake.dropdownState = robot.intake.dropdownState.previous();
            while (robot.intake.pixelCount() < 2 &&
                    intakeTimer.seconds() < intakeTimerLimit / 2.0
                    && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }
            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit &&
                    opModeIsActive() && !isStopRequested()) {
                intakeTimer.reset();
                while (robot.intake.pixelCount() < 2 &&
                        intakeTimer.seconds() < intakeTimerLimit
                        && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
                if (robot.intake.pixelCount() == 2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                } else {
                    robot.intake.dropdownState = robot.intake.dropdownState.previous();
                }
            }
            Intake.intakeSensorsOn = false;
            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;

            // 9 -> go to backdrop
            robot.drive.followTrajectory(trajectories.get(9));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
                }

                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                }

                if (0.6 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.8) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                }

                if (robot.drive.getPoseEstimate().getX() > 9) {
                    robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                }
                robot.sleep(0.01);
            }
        }
        robot.drive.followTrajectory(trajectories.get(10));
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                robot.outtake.rotateState = Outtake.RotateState.CENTER;
            }
            robot.sleep(0.01);
        }

        robot.intake.dropdownState = Intake.DropdownState.UP;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(0.1);

        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.clawState = Outtake.ClawState.OPEN;

        robot.sleep(0.5);
        robot.stop();
    }
}
