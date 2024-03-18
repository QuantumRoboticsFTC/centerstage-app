package eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

// Red Audience Centerstage
public class TrajectoryRAWall {
    public static Pose2d START_POSE = new Pose2d(-38.1, -63.5, Math.toRadians(270));

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

    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0 -> left
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-48, -34), Math.toRadians(90))
                .build()
        );
        // 1 -> center
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-40, -35), Math.toRadians(90))
                .build()
        );
        // 2 -> right
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-25, -36), Math.toRadians(0))
                .build()
        );

        // 3 -> center2Stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-37, -47), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(0))
                .build()
        );

        // 4 -> go to lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-40, -60))
                .build()
        );

        // 5 -> go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(30, -60))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(0))
                .build()
        );

        // 6 -> go to lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -60))
                .build()
        );

        // 7 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-40, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))
                .build()
        );

        // 8 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build()
        );
        return trajectories;
    }
}