package eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
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

public class TrajectoryCloseRed {
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
    public static List<Trajectory> centerstageTrajectories(Robot robot, int cycleCount, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(9, -41, Math.toRadians(320)))
                    .addTemporalMarker(0.15, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(14, -42, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(49, -36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(23, -31, Math.toRadians(320)))
                    .addTemporalMarker(0.15, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(26, -35, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(49, -36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(30, -40, Math.toRadians(320)))
                    .addTemporalMarker(0.15, () -> {
                        robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    })
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(320), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(49, -36.5, Math.toRadians(0)))
                    .build()
            );
        }

        // cycleurile
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(41, -17, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-15, -13), Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(-55, -16, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(0.2, () -> {
                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                    robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                })
                .addTemporalMarker(1.75, () -> {
                    robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                })
                .build()
        );

        for (int i = 1; i < cycleCount; i++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-57, -15, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(-60, -15), Math.toRadians(180))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(20), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-54, -14, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(15, -14), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(49, -22, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(0.2, () -> {
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    })
                    .addTemporalMarker(0.9, () -> {
                        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                        robot.outtake.clawState = Outtake.ClawState.CLOSED
                        ;
                    })
                    .addTemporalMarker(1.5, () -> {
                        robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                        robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
                    })
                    .addTemporalMarker(1.85, () -> {
                        robot.outtake.diffyHState = Outtake.DiffyHortizontalState.RIGHT;
                        robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                        robot.elevator.targetHeight = Elevator.TargetHeight.THIRD_LINE;
                    })
                    .build()
            );

            if (i != cycleCount) {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(33, -17, Math.toRadians(0)))
                        .splineToConstantHeading(new Vector2d(-15, -14), Math.toRadians(160))
                        .splineToSplineHeading(new Pose2d(-55, -16, Math.toRadians(0)), Math.toRadians(180))
                        .addTemporalMarker(0.3, () -> {
                            robot.outtake.rotateState = Outtake.RotateState.CENTER;
                            robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
                            robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                        })
                        .addTemporalMarker(1.75, () -> {
                            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                        })
                        .build()
                );
            }
        }


        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -60))
                    .splineToConstantHeading(new Vector2d(57.5, -60), Math.toRadians(180))
                    .build()
            );
        } else {
            // parked left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -13))
                    .build()
            );
        }
        return trajectories;
    }

    // cica aproape incepute
    public static List<Trajectory> trussTrajectories(Robot robot, int cycleCount, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(7, -41, Math.toRadians(140)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(140), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(14, -42, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(23.5, -28, Math.toRadians(140)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(140), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(26, -35, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(30, -40, Math.toRadians(140)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(140), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(38, -42, Math.toRadians(160)))
                    .splineToSplineHeading(new Pose2d(48, -42.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        }

        // cycleurile
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(40, -48, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-25, -59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57, -40, Math.toRadians(200)), Math.toRadians(140))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(200), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-59, -37, Math.toRadians(160)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(200), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-55, -44, Math.toRadians(140)))
                .splineToSplineHeading(new Pose2d(5, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(180)), Math.toRadians(30))
                .build()
        );

        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -60))
                    .splineToConstantHeading(new Vector2d(62, -60), Math.toRadians(0))
                    .build()
            );
        } else {
            // parked left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -13))
                    .build()
            );
        }
        return trajectories;
    }

    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, int teamProp, boolean truss, boolean parkedRight) {
        if (truss) {
            return trussTrajectories(robot, cycleCount, teamProp, parkedRight);
        } else {
            return centerstageTrajectories(robot, cycleCount, teamProp, parkedRight);
        }
    }

}