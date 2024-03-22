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
    public static Pose2d START_POSE = new Pose2d(-37, -63, Math.toRadians(270));

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
                .lineToConstantHeading(new Vector2d(-48, -34))
                .build()
        );
        // 1 -> left to "prepare for stack"
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(280), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-36, -43, Math.toRadians(0)))
                .build()
        );

        // 2 -> center
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-38, -34, Math.toRadians(280)))
                .build()
        );
        // 3 -> center to "prepare for stack"
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(260), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-36, -43, Math.toRadians(0)))
                .build()
        );

        // 4 -> right
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-36, -33, Math.toRadians(190)))
                .build()
        );
        // 5 -> right to "prepare for stack"
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(190), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-50, -35, Math.toRadians(0)))
                .build()
        );

        // 6 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-60, -36))
                .build()
        );

        // 7 -> go to lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-40, -60))
                .build()
        );

        // 8 -> go to backboard
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(30, -60))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(0))
                .build()
        );

        // 9 -> go to lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -60))
                .build()
        );

        // 10 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-40, -60))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))
                .build()
        );

        // 11 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build()
        );
        return trajectories;
    }
}