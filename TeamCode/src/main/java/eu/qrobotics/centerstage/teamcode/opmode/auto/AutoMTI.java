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
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryMTI;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryRBWall_2_4;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name="AutoMTI",group = "main")
public class AutoMTI extends LinearOpMode {
    public Robot robot;
    List<Trajectory> trajectories;
    private VisionPortal visionPortalTeamProp;
    private TeamPropDetectionRed teamPropDetectionRed;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera
    int teamProp = -1;
    public static int cycleCount = 2;
    int trajectoryId = 0;
    public static double MAX_DISTANCE = -100;

    ElapsedTime autoTimer = new ElapsedTime(50);
    ElapsedTime bigIntakeTimer = new ElapsedTime(50);
    ElapsedTime intakeTimer = new ElapsedTime(50);
    ElapsedTime trajectoryTimer = new ElapsedTime(50);
    double timerLimit1 = 28;
    double timerLimit2 = 28;
    double intakeTimerLimit = 0.4;
    double bigIntakeTimerLimit = 2.0;


    void solvePurplePixel() {
        robot.drive.followTrajectory(trajectories.get(trajectoryId++));
       /* if (teamProp == 1) {
            // left
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        } else if (teamProp == 3) {
            // right
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        } else {
            // center
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        }*/
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.outtake.rotateState = Outtake.RotateState.RIGHT;
        trajectoryTimer.reset();
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
        robot.sleep(0.1);
    }

    void placePixel() {
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.25);
    }

    void retryOuttake() {
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
        robot.drive.followTrajectory(trajectories.get(13));
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
        robot.drive.setPoseEstimate(TrajectoryRBWall_2_4.START_POSE);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
        robot.outtake.clawState = Outtake.ClawState.CLOSED;


        trajectories= TrajectoryMTI.getTrajectories();

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

        teamProp=2;

        solvePurplePixel();

        // 1 -> go to backdrop

        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
        robot.sleep(0.2);
        trajectoryTimer.reset();
        robot.drive.followTrajectory(trajectories.get(trajectoryId++));
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
            robot.sleep(0.01);

            if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.25) {
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            }
        }
        robot.sleep(0.1);


        for(int cycle=1;cycle<=3;cycle++){
            placePixel();
            telemetry.addLine("Cycle "+String.valueOf(cycle)+" runing");
            telemetry.update();
            robot.outtake.rotateState = Outtake.RotateState.CENTER;
            robot.sleep(0.2);

            //go to stack
            robot.drive.followTrajectory(trajectories.get(trajectoryId++));
            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                if (robot.drive.getPoseEstimate().getX() < 43 && robot.drive.getPoseEstimate().getX() > 0) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    if (cycle == 1||cycle==3) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
                    } else if (cycle == 2) {
                        robot.intake.dropdownState = Intake.DropdownState.STACK_3;
                    }
                } else if (robot.drive.getPoseEstimate().getX() < -20) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                }
                robot.sleep(0.01);
            }

            if(cycle==3){
                robot.drive.followTrajectory(trajectories.get(trajectoryId++));
                trajectoryTimer.reset();
                while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }
            }

            //at stack
            robot.sleep(0.5);
            if(robot.intake.dropdownState == Intake.DropdownState.STACK_5){
                robot.intake.dropdownState = Intake.DropdownState.STACK_4;
            }
            else if(robot.intake.dropdownState == Intake.DropdownState.STACK_3){
                robot.intake.dropdownState = Intake.DropdownState.STACK_2;
            }
            robot.sleep(0.5);
            robot.intake.intakeMode = Intake.IntakeMode.OUT;

            // go to backdrop
            robot.drive.followTrajectory(trajectories.get(trajectoryId++));

            trajectoryTimer.reset();
            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
                if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.3) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                }
                if (robot.drive.getPoseEstimate().getX() > 5) {
                    if (cycle != 3) {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    } else {
                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    }
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    if (cycle == 1 || cycle == 3) {
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                    }else if (cycle == 2){
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                    }
                }
                if (robot.drive.getPoseEstimate().getX() > 30) {
                    robot.outtake.rotateState= Outtake.RotateState.RIGHT;

                }
                robot.sleep(0.01);
            }
            telemetry.addLine("Cycle "+String.valueOf(cycle)+" done");
            telemetry.update();
        }

        telemetry.addLine("Parking");
        telemetry.update();
        //park
        robot.drive.followTrajectory(trajectories.get(trajectoryId++));
        telemetry.addLine("Parked");
        telemetry.update();

//        for(Trajectory trajectory:trajectories) {
//            robot.drive.followTrajectory(trajectory);
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
//                robot.sleep(0.01);
//            }
//            robot.sleep(0.5);
//        }

//
//        solvePurplePixel();
//        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
//        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT0;
//        robot.sleep(0.2);
//
//        // 1 -> go to initial backdrop
//        trajectoryTimer.reset();
//        robot.drive.followTrajectory(trajectories.get(1));
//        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
//            robot.sleep(0.01);
//
//            if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.25) {
//                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
//                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
//            }
//        }
//        robot.sleep(0.1);
//        placePixel();
//
//        // TODO: *cica* cycles
//        for (int i = 1; i <= cycleCount; i++) {
//            if (i == 1) {
//                // 2 -> initial go to lane
//                robot.drive.followTrajectory(trajectories.get(2));
//            } else {
//                // 7 -> go to lane after drop
//                robot.drive.followTrajectory(trajectories.get(7));
//            }
//
//            trajectoryTimer.reset();
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//                if (0.2 < trajectoryTimer.seconds() && robot.drive.getPoseEstimate().getX() >= 43) {
//                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
//                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
//                }
//                if (robot.drive.getPoseEstimate().getX() < 43) {
//                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
//                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//                    if (i == 1 || i == 3) {
//                        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
//                    } else if (i == 2) {
//                        robot.intake.dropdownState = Intake.DropdownState.STACK_3;
//                    }
//                }
//                robot.sleep(0.01);
//            }
//            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//
//            if (i == 3) {
//                // se duce la al 2lea stack
//                break;
//            }
//
//            if (i == 1) {
//                // 7 -> go to lane before stack
//                robot.drive.followTrajectory(trajectories.get(3));
//            } else {
//                // 12 -> go to lane before stack
//                robot.drive.followTrajectory(trajectories.get(8));
//            }
//            trajectoryTimer.reset();
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
//                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//                }
//                robot.sleep(0.01);
//            }
//            if (i == 1) {
//                // 8 -> go to stack
//                robot.drive.followTrajectory(trajectories.get(4));
//            } else {
//                // 13 -> go to stack
//                robot.drive.followTrajectory(trajectories.get(9));
//            }
//            trajectoryTimer.reset();
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.5) {
//                    robot.intake.intakeMode = Intake.IntakeMode.IN;
//                    Intake.intakeSensorsOn = true;
//                }
//                robot.sleep(0.01);
//            }
//
////            startLineFollower();
//            bigIntakeTimer.reset();
//            intakeTimer.reset();
//            while (robot.intake.pixelCount() < 2 &&
//                    intakeTimer.seconds() < intakeTimerLimit
//                    && opModeIsActive() && !isStopRequested()) {
//                telemetry.addData("pixel count", robot.intake.pixelCount());
//                telemetry.update();
//                robot.sleep(0.01);
//            }
//            robot.intake.dropdownState = robot.intake.dropdownState.previous();
//            while (robot.intake.pixelCount() < 2 &&
//                    intakeTimer.seconds() < intakeTimerLimit
//                    && opModeIsActive() && !isStopRequested()) {
//                telemetry.addData("pixel count", robot.intake.pixelCount());
//                telemetry.update();
//                robot.sleep(0.01);
//            }
//            while (robot.intake.pixelCount() < 2 && bigIntakeTimer.seconds() < bigIntakeTimerLimit &&
//                    opModeIsActive() && !isStopRequested()) {
//                intakeTimer.reset();
//                while (robot.intake.pixelCount() < 2 &&
//                        intakeTimer.seconds() < intakeTimerLimit
//                        && opModeIsActive() && !isStopRequested()) {
//                    telemetry.addData("pixel count", robot.intake.pixelCount());
//                    telemetry.update();
//                    robot.sleep(0.01);
//                }
//                if (robot.intake.pixelCount() == 2) {
//                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
//                } else {
//                    robot.intake.dropdownState = robot.intake.dropdownState.previous();
//                }
//                telemetry.addData("pixel count", robot.intake.pixelCount());
//                telemetry.update();
//            }
//            Intake.intakeSensorsOn = false;
//            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
//
//            if (i == 1) {
//                // 9 -> go under truss
//                robot.drive.followTrajectory(trajectories.get(5));
//            } else {
//                // 14 -> go under truss
//                robot.drive.followTrajectory(trajectories.get(10));
//            }
//            trajectoryTimer.reset();
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//                if (0.1 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.2) {
//                    robot.intake.dropdownState = Intake.DropdownState.ALMOST_UP;
//                }
//
//                if (0.3 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
//                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
//                }
//                robot.sleep(0.01);
//            }
//            robot.intake.intakeMode = Intake.IntakeMode.IN;
////            aTagDetector.detect();
////            if (aTagDetector.detected) {
////                robot.drive.setPoseEstimate(aTagDetector.estimatedPose);
////            }
////            robot.sleep(0.4);
//
//            if (i == 1) {
//                // 10 -> go to backdrop
//                robot.drive.followTrajectory(trajectories.get(6));
//            } else {
//                // 15 -> go to backdrop
//                robot.drive.followTrajectory(trajectories.get(11));
//            }
//
//            Intake.intakeSensorsOn = true;
//            trajectoryTimer.reset();
//            while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested() && robot.outtake.getMeanSensorDistance() >= MAX_DISTANCE) {
//                if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.3) {
//                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
//                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
//                    robot.intake.dropdownState = Intake.DropdownState.UP;
//                }
//                if (robot.drive.getPoseEstimate().getX() > 5) {
//                    if (i == 1) {
//                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
//                    } else if (i == 2) {
//                        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
//                    }
//                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
//                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
//                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
//                }
//                if (robot.drive.getPoseEstimate().getX() > 30 &&
//                        robot.intake.pixelCount() == 2) {
//                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//                    robot.outtake.clawState = Outtake.ClawState.OPEN;
//                }
//                robot.sleep(0.01);
//            }
//            if (robot.intake.pixelCount() != 2) {
//                robot.sleep(0.2);
//                placePixel();
//            }
//            for (int tryCount = 0; tryCount <= 2 && robot.intake.pixelCount() > 0; tryCount++) {
//                retryOuttake();
//            }
//            Intake.intakeSensorsOn = false;
//        }
//        robot.drive.followTrajectory(trajectories.get(14));
//        trajectoryTimer.reset();
//        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {
//            if (0.2 < trajectoryTimer.seconds() && trajectoryTimer.seconds() < 0.4) {
//                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
//                robot.outtake.rotateState = Outtake.RotateState.CENTER;
//            }
//            robot.sleep(0.01);
//        }
//
//        robot.intake.dropdownState = Intake.DropdownState.UP;
//        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//        robot.sleep(0.1);
//
//        robot.outtake.rotateState = Outtake.RotateState.CENTER;
//        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
//        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//        robot.outtake.clawState = Outtake.ClawState.OPEN;

        robot.sleep(0.5);
        robot.stop();
    }
}
