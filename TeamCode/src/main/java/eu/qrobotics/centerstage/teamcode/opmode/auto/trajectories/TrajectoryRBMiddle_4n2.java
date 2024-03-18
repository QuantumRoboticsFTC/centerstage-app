package eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
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

// Red Backboard Centerstage
public class TrajectoryRBMiddle_4n2 {
    public static Pose2d START_POSE = new Pose2d(14.1, -63.5, Math.toRadians(90));

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
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(12, -36, Math.toRadians(180)), Math.toRadians(120))
                .build()
        );
        // 1 -> center
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(18, -26, Math.toRadians(180)), Math.toRadians(90))
                .build()
        );
        // 2 -> right
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(32, -35.5, Math.toRadians(180)), Math.toRadians(90))
                .build()
        );

        // 3 -> initial backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(180)))
                .build()
        );

        // 4 -> initial go to lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(40, -12))
                .build()
        );

        // 5 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-58, -13))
                .build()
        );

        // 6 -> go to lane in front of drop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -12))
                .build()
        );

        // 7 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(48, -21.5))
                .build()
        );

        // 8 -> go to lane after drop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -12))
                .build()
        );

        // 9 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(58, -14), Math.toRadians(0))
                .build()
        );

        // 10 -> dummy
//        // 7 -> go to lane after drop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -12))
                .build()
        );

        // 11 -> go to lane before stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-42, -12))
                .build()
        );

        // 12 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-58.5, -23.5, Math.toRadians(190)))
                .build()
        );

        // 13 -> go to lane after stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(190), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)))
                .build()
        );

        // 7/14 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -12))
                .splineToConstantHeading(new Vector2d(48, -17), Math.toRadians(0))
                .build()
        );

        // 15 - dummy
        // finish @ lane (near backdrop)
        // 6 -> go to lane in front of drop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(35, -12))
                .build()
        );

        // 16 -> go directly to park (from lane)
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(58, -14))
                .build()
        );

        return trajectories;
    }
}