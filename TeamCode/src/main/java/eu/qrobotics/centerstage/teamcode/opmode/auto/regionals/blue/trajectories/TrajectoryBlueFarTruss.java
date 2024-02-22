package eu.qrobotics.centerstage.teamcode.opmode.auto.regionals.blue.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.ZOOM_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.ZOOM_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

public class TrajectoryBlueFarTruss {
    public static Pose2d START_POSE = new Pose2d(-38.1, 63.5, Math.toRadians(90));

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-40, 31.5, Math.toRadians(30)))
                    .build()
            );
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(30), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-36.5, 41, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(-60.75, 37.5), Math.toRadians(120))
                    .addTemporalMarker(0.3, ()->{robot.intake.intakeMode = Intake.IntakeMode.IDLE;})
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-39, 35.75, Math.toRadians(80)))
                    .build()
            );
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(80), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-34.5, 36.5, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(-60.75, 37.5), Math.toRadians(120))
                    .addTemporalMarker(0.3, ()->{robot.intake.intakeMode = Intake.IntakeMode.IDLE;})
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-35, 32.5, Math.toRadians(180)))
                    .build()
            );
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-50, 41, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(-60.75, 37.5), Math.toRadians(135))
                    .addTemporalMarker(0.3, ()->{robot.intake.intakeMode = Intake.IntakeMode.IDLE;})
                    .build()
            );
        }

        // cycleurile
        // first goto pixels
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-49, 38), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-53.5, 37), Math.toRadians(0))
                .addTemporalMarker(0.2, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                })
                .addTemporalMarker(0.45, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                })
                .addTemporalMarker(0.6, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                })
                .addTemporalMarker(0.9, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                })
                .build()
        );

        // go to good path
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-50, 60))
                .build()
        );

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.25, 40), Math.toRadians(0))
                .addTemporalMarker(0.4, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                    robot.elevator.manualPower = -1;
                })
                .addTemporalMarker(1.0, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                })
                .addTemporalMarker(1.65, () -> {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .addTemporalMarker(1.75, () -> {
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
//                    robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                })
                .build()
        );

        // go to good path
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(33, 60))
                .build()
        );

        // go to pixel stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-36, 60))
                .splineToConstantHeading(new Vector2d(-52, 43), Math.toRadians(180))
                .addTemporalMarker(0.05, () -> {
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .build()
        );

        // CYCLE 2
        // intake pixel form stack FIRST
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-61, 37.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-49, 39), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-53.5, 37.5), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                })
                .addTemporalMarker(0.25, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                })
                .addTemporalMarker(0.45, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                })
                .build()
        );

        // intake pixel form stack SECOND
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-61, 37.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-49, 39), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-53.5, 37.5), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                })
                .addTemporalMarker(0.25, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                    robot.intake.intakeMode = Intake.IntakeMode.IN;
                })
                .addTemporalMarker(0.3, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                })
                .addTemporalMarker(0.45, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.DOWN;
                })
                .build()
        );

        // go to good path
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-50, 60))
                .build()
        );

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.25, 48), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                    robot.elevator.manualPower = -1;
                })
                .addTemporalMarker(1.0, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                })
                .addTemporalMarker(1.6, () -> {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .addTemporalMarker(1.75, () -> {
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                    robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
                    robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT2;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                })
                .build()
        );

        // parked right
        if (!parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(42, 56.5))
                    .splineToConstantHeading(new Vector2d(53.5, 60), Math.toRadians(180))
                    .addTemporalMarker(0.15, () -> {
                        robot.outtake.rotateState = Outtake.RotateState.CENTER;
                        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    })
                    .build()
            );
        } else {
            // parked left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(40, 30))
                    .splineToConstantHeading(new Vector2d(52.5, 11), Math.toRadians(180))
                    .addTemporalMarker(0.1, () -> {
                        robot.outtake.rotateState = Outtake.RotateState.CENTER;
                        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    })
                    .build()
            );
        }

        // debug Trajectory 1 <-> if no sliders and shit, reposition and try again
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineTo(new Vector2d(-12, 60))
                .splineToConstantHeading(new Vector2d(15, 57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.25, 37.5), Math.toRadians(0))
                .addTemporalMarker(0.35, () -> {
                    if (5 <= robot.drive.getLocalizer().getPoseEstimate().getX()) {
                        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                        robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    }
                })
                .build()
        );

        // debug Trajectory 2 <-> pentru cand mergem la pixeli
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-12, 60))
                .splineToConstantHeading(new Vector2d(-30, 57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52, 43), Math.toRadians(180))
                .build()
        );

        return trajectories;
    }
}