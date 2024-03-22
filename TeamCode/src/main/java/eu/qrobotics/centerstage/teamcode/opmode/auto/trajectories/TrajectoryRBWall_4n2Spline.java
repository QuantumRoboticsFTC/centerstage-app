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

    public static List<Trajectory> getTrajectories(Robot robot, int cycleCount, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0 -> left
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(0)))
                .build()
        );

        // 1 -> dummy node
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(13, -63, Math.toRadians(270)))
                .build()
        );
        // 2 -> center
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
//        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(18, -26, Math.toRadians(0)))
                .build()
        );

        // 3 -> dummy node
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(13, -63, Math.toRadians(270)))
                .build()
        );
        // 4 -> right
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(32, -35.5, Math.toRadians(0)))
                .build()
        );


        // 5 -> initial backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(47.5, -35, Math.toRadians(0)))
                .build()
        );

        // 6 -> initial go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -45))
                .splineToConstantHeading(new Vector2d(-48, -60), Math.toRadians(190))
                .splineToConstantHeading(new Vector2d(-58.5, -37), Math.toRadians(180))
                .build()
        );

        // 7 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-45, -50))
                .splineToConstantHeading(new Vector2d(47, -50.5), Math.toRadians(0))
                .build()
        );

        // 8 -> 2nd go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -45))
                .splineToConstantHeading(new Vector2d(-48, -60), Math.toRadians(190))
                .splineToConstantHeading(new Vector2d(-58.5, -37), Math.toRadians(180))
                .build()
        );

        // 9 -> dummy
        // 7 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-45, -50))
                .splineToConstantHeading(new Vector2d(47, -50.5), Math.toRadians(0))
                .build()
        );

        // 10 -> 3rd go to stack (2nd stack)
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -53))
                .splineToConstantHeading(new Vector2d(-43, -60), Math.toRadians(190))
                .splineToSplineHeading(new Pose2d(-59.5, -28, Math.toRadians(335)), Math.toRadians(90))
                .build()
        );

        // 11 -> 2nd ting go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(335), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-48, -54, Math.toRadians(0)), Math.toRadians(350))
                .splineToSplineHeading(new Pose2d(47, -50.5, Math.toRadians(0)), Math.toRadians(30))
                .build()
        );

        // 12 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
//        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                        .splineToConstantHeading(new Vector2d(58, -58), Math.toRadians(0))
                        .build()
        );
        return trajectories;
    }
}