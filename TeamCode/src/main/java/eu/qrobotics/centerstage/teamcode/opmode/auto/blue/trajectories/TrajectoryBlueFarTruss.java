package eu.qrobotics.centerstage.teamcode.opmode.auto.blue.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

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
                    .lineToSplineHeading(new Pose2d(-39, 33.5, Math.toRadians(30)))
                    .build()
            );
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(330), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-33.5, 38, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(-60.75, 37.5), Math.toRadians(120))
                    .addTemporalMarker(0.3, ()->{robot.intake.intakeMode = Intake.IntakeMode.IDLE;})
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-41, 33.5, Math.toRadians(50)))
                    .build()
            );
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(50), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
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
                .splineToConstantHeading(new Vector2d(-49, 39), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-53.5, 37.5), Math.toRadians(0))
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

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-45, 55))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.25, 37.5), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                    robot.elevator.manualPower = -1;
                })
                .addTemporalMarker(1.2, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                })
                .addTemporalMarker(1.4, () -> {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .addTemporalMarker(1.9, () -> {
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
                })
                .addTemporalMarker(2.0, () -> {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                })
                .build()
        );

        // go to pixel stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(41, 41))
                .splineToConstantHeading(new Vector2d(30, 61), Math.toRadians(190))
                .splineToConstantHeading(new Vector2d(-20, 61), Math.toRadians(190))
                .splineToConstantHeading(new Vector2d(-52, 43), Math.toRadians(180))
                .addTemporalMarker(0.35, () -> {
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

        // intake pixel form stack SECOND
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-61, 37.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-49, 39), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52.75, 37.5), Math.toRadians(0))
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

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-45, 55))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.25, 43.5), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    robot.intake.dropdownState = Intake.DropdownState.UP;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                    robot.elevator.manualPower = -1;
                })
                .addTemporalMarker(1.2, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                })
                .addTemporalMarker(1.4, () -> {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .addTemporalMarker(1.9, () -> {
                    robot.outtake.rotateState = Outtake.RotateState.LEFT;
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
                })
                .addTemporalMarker(2.0, () -> {
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
                })
                .build()
        );

        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(42, 56.5))
                    .splineToConstantHeading(new Vector2d(53.5, 58), Math.toRadians(180))
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
                    .lineTo(new Vector2d(44, 27))
                    .splineToConstantHeading(new Vector2d(52.5, 13), Math.toRadians(180))
                    .addTemporalMarker(0.1, () -> {
                        robot.outtake.rotateState = Outtake.RotateState.CENTER;
                        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                    })
                    .build()
            );
        }
        return trajectories;
    }
}