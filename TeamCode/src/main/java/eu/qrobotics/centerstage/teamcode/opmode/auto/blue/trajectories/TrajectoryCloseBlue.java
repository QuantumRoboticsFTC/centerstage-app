package eu.qrobotics.centerstage.teamcode.opmode.auto.blue.trajectories;

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

import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

public class TrajectoryCloseBlue {
    public static Pose2d START_POSE = new Pose2d(14.1, 63.5, Math.toRadians(90));;

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
    public static List<Trajectory> centerstageTrajectories(Robot robot, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(9, 41, Math.toRadians(40)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(40), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(14, 42, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(48, 36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(23.5, 28, Math.toRadians(40)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(40), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(26, 35, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(48, 36.5, Math.toRadians(0)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(30, 40, Math.toRadians(40)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(40), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(48, 36.5, Math.toRadians(0)))
                    .build()
            );
        }

        // cycleurile
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(41, 23, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-15, 12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57, 10, Math.toRadians(20)), Math.toRadians(180))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(20), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-59, 13, Math.toRadians(340)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(340), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-54, 12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(15, 14, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 20, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0.4, () -> {robot.outtake.clawState = Outtake.ClawState.CLOSED;})
                .addTemporalMarker(1.5, () -> {
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                    robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
                })
                .addTemporalMarker(1.85, () -> {
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
                })
                .build()
        );

        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, 60))
                    .splineToConstantHeading(new Vector2d(62, 60), Math.toRadians(0))
                    .build()
            );
        } else {
            // parked left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, 13))
                    .build()
            );
        }
        return trajectories;
    }

    // cica aproape incepute
    public static List<Trajectory> trussTrajectories(Robot robot, int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(7, 41, Math.toRadians(220)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(220), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(14, 42, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(48, 36.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(23.5, 28, Math.toRadians(220)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(220), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(26, 35, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(48, 36.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(30, 40, Math.toRadians(220)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(220), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(38, 42, Math.toRadians(200)))
                    .splineToSplineHeading(new Pose2d(48, 42.5, Math.toRadians(180)), Math.toRadians(30))
                    .build()
            );
        }

        // cycleurile
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(40, 48, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-25, 59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57, 40, Math.toRadians(160)), Math.toRadians(140))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(160), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-59, 37, Math.toRadians(200)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(160), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-55, 44, Math.toRadians(220)))
                .splineToSplineHeading(new Pose2d(5, 60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 48, Math.toRadians(180)), Math.toRadians(30))
                .build()
        );

        // parked right
        if (parkedRight) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, 60))
                    .splineToConstantHeading(new Vector2d(62, 60), Math.toRadians(0))
                    .build()
            );
        } else {
            // parked left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, 13))
                    .build()
            );
        }
        return trajectories;
    }

    public static List<Trajectory> getTrajectories(Robot robot, int teamProp, boolean truss, boolean parkedRight) {
        if (truss) {
            return trussTrajectories(robot, teamProp, parkedRight);
        } else {
            return centerstageTrajectories(robot, teamProp, parkedRight);
        }
    }

}