package eu.qrobotics.centerstage.teamcode.opmode.auto.red.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.NORMAL_VEL_CONSTRAINT;
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

public class TrajectoryRedCloseCS {
    public static Pose2d START_POSE = new Pose2d(14.1, -63.5, Math.toRadians(270));

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

    // cica gata
    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(9, -41, Math.toRadians(320)))
                    .addTemporalMarker(0.1, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(14, -42, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(47.25, -36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(23, -32.5, Math.toRadians(320)))
                    .addTemporalMarker(0.1, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(26, -35, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(47.25, -36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(31, -40, Math.toRadians(320)))
                    .addTemporalMarker(0.1, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(47.25, -36.5, Math.toRadians(0)))
                    .build()
            );
        }

        // cycleurile
        // first goto pixels
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(41, -15))
                .splineToConstantHeading(new Vector2d(-15, -15), Math.toRadians(160))
                .splineToConstantHeading(new Vector2d(-52, -15), Math.toRadians(180))
                .addTemporalMarker(0.2, () -> {
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .build()
        );

        // CYCLE 1
        // intake pixel form stack FIRST
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-59, -15.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
            .splineToConstantHeading(new Vector2d(-49, -17), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(-52, -15.5), Math.toRadians(0))
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
            .addTemporalMarker(0.75, () -> {
                robot.intake.dropdownState = Intake.DropdownState.DOWN;
            })
            .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-59, -14.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
            .splineToConstantHeading(new Vector2d(-49, -16), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(-52.75, -14.5), Math.toRadians(0))
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
            .addTemporalMarker(0.75, () -> {
                robot.intake.dropdownState = Intake.DropdownState.DOWN;
            })
            .build()
        );

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
            .lineToConstantHeading(new Vector2d(-45, -14))
            .splineToConstantHeading(new Vector2d(15, -14), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(47.25, -28), Math.toRadians(0))
            .addTemporalMarker(0.15, () -> {
                robot.intake.dropdownState = Intake.DropdownState.UP;
            })
            .addTemporalMarker(0.65, () -> {
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                robot.elevator.manualPower = -1;
            })
            .addTemporalMarker(1.2, () -> {
                robot.outtake.clawState = Outtake.ClawState.CLOSED;
                robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
            })
            .addTemporalMarker(1.5, () -> {
                robot.outtake.rotateState = Outtake.RotateState.RIGHT;
                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
            })
            .addTemporalMarker(2.0, () -> {
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
            })
            .build()
        );

        // go to pixel stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
            .lineToConstantHeading(new Vector2d(28, -14))
            .splineToConstantHeading(new Vector2d(-15, -12.5), Math.toRadians(160))
            .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(180))
            .addTemporalMarker(0.3, () -> {
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
                .lineToConstantHeading(new Vector2d(-59, -14.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
            .splineToConstantHeading(new Vector2d(-49, -15.5), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(-52, -13), Math.toRadians(0))
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
            .addTemporalMarker(0.75, () -> {
                robot.intake.dropdownState = Intake.DropdownState.DOWN;
            })
            .build()
        );

        // intake pixel form stack SECOND
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-58.8, -14.5))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
            .splineToConstantHeading(new Vector2d(-49, -16), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(-51, -14.5), Math.toRadians(0))
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
            .addTemporalMarker(0.75, () -> {
                robot.intake.dropdownState = Intake.DropdownState.DOWN;
            })
            .build()
        );

        // go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
            .lineToConstantHeading(new Vector2d(-45, -14.5))
            .splineToConstantHeading(new Vector2d(15, -14.5), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(48, -28), Math.toRadians(0))
            .addTemporalMarker(0.15, () -> {
                robot.intake.dropdownState = Intake.DropdownState.UP;
            })
            .addTemporalMarker(0.65, () -> {
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
                robot.elevator.manualPower = -1;
            })
            .addTemporalMarker(1.2, () -> {
                robot.outtake.clawState = Outtake.ClawState.CLOSED;
                robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
            })
            .addTemporalMarker(1.5, () -> {
                robot.outtake.rotateState = Outtake.RotateState.RIGHT;
                robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                robot.outtake.manualFourbarPos = Outtake.FOURBAR_SCORE_POS;
            })
            .addTemporalMarker(2.0, () -> {
                robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HEIGHT1;
            })
            .build()
        );


        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(46, -64))
                    .splineToConstantHeading(new Vector2d(56, -64), Math.toRadians(0))
                    .addTemporalMarker(0.25, () -> {
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
                    .lineTo(new Vector2d(39, -23))
                    .splineToConstantHeading(new Vector2d(55, -13), Math.toRadians(0))
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