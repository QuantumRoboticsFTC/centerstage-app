package eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories;

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

import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Backboard Wall
public class TrajectoryRBWall_4n2Spline {
    public static Pose2d START_POSE = new Pose2d(13, -63, Math.toRadians(270));

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

    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, boolean parkedRight, int teamProp) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            // 0 -> left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(10.75, -34.5, Math.toRadians(0)))
                    .build()
            );
            // 1 -> initial backdrop
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                    .build()
            );
        } else if (teamProp == 2 || teamProp == -1) {
            // 0 -> center
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(18, -25, Math.toRadians(0)))
                    .build()
            );
            // 1 -> initial backdrop
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(48, -45.5, Math.toRadians(0)))
                    .build()
            );
        } else if (teamProp == 3) {
            // 0 -> right
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(31, -34, Math.toRadians(0)))
                    .build()
            );
            // 1 -> initial backdrop
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))
                    .build()
            );
        }

        // 2 -> initial go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -59))
                .splineToConstantHeading(new Vector2d(-15, -63.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.5, -37), Math.toRadians(180))
                .build()
        );

        // 3 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-42, -52))
                .splineToConstantHeading(new Vector2d(47, -50.5), Math.toRadians(0))
                .build()
        );

        // 4 -> 2nd go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -59))
                .splineToConstantHeading(new Vector2d(-15, -63.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.5, -37), Math.toRadians(180))
                .build()
        );

        // 5 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-42, -52))
                .splineToConstantHeading(new Vector2d(47, -50.5), Math.toRadians(0))
                .build()
        );

        // 6 -> go to lane // 2nd stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -56))
                .build()
        );

        // 7 -> go to lane before stack // 2nd stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-45, -55))
                .build()
        );

        // 8 -> go to stack // 2nd stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(335), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-60.5, -28, Math.toRadians(335)))
                .build()
        );

        // 9 -> go to backdrop // 2nd stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(30, -60))
                .splineToConstantHeading(new Vector2d(47, -50.5), Math.toRadians(0))
                .build()
        );

        // 10 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                        .splineToConstantHeading(new Vector2d(58, -58), Math.toRadians(0))
                        .build()
        );
        return trajectories;
    }
}