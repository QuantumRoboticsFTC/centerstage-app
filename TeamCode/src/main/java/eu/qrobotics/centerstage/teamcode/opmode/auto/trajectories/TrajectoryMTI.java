package eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

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

public class TrajectoryMTI {
    public static Pose2d START_POSE = new Pose2d(13, -63, Math.toRadians(270));
    public static TrajectoryVelocityConstraint startVelocity=BASE_VEL_CONSTRAINT;
    public static TrajectoryAccelerationConstraint startAcceleration =BASE_ACCEL_CONSTRAINT;

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, startVelocity, startAcceleration);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, startVelocity, startAcceleration);
    }

    public static List<Trajectory> getTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();


        // 0 -> center
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(270), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(18, -24, Math.toRadians(0)))
                .build()
        );
        // 1 -> initial backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(0)))
                .build()
        );

        // 2 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-47.5, -38))
                .splineToConstantHeading(new Vector2d(-56.7, -36),Math.toRadians(0),NORMAL_VEL_CONSTRAINT,NORMAL_ACCEL_CONSTRAINT)
                .build()
        );

        // 3 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0),  BASE_VEL_CONSTRAINT,  BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(27, -36, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50.5, -38), Math.toRadians(0), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                .build()
        );

        // 4 -> go to stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-50, -37))
                .splineToConstantHeading(new Vector2d(-57.2, -36),Math.toRadians(0),NORMAL_VEL_CONSTRAINT,NORMAL_ACCEL_CONSTRAINT)
                .build()
        );

        // 5 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0),  ZOOM_VEL_CONSTRAINT,  ZOOM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(27, -36, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50.5, -33.5),Math.toRadians(0),NORMAL_VEL_CONSTRAINT,NORMAL_ACCEL_CONSTRAINT)
                .build()
        );

        // 6 -> go to secondary stack pt. 1
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0),ZOOM_VEL_CONSTRAINT ,ZOOM_ACCEL_CONSTRAINT )
                .lineToConstantHeading(new Vector2d(35, -36))
                .build()
        );

        // 7 -> go to secondary stack pt. 2
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-35, -36))
                .build()
        );


        // 8 -> go to secondary stack pt. 3
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-59, -24))
                .build()
        );


        // 9 -> back on lane
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-35, -36),Math.toRadians(0),ZOOM_VEL_CONSTRAINT,ZOOM_ACCEL_CONSTRAINT)
                .build()
        );

        // 10 -> go to backdrop
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(27, -38, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50.5, -33.5),Math.toRadians(0),NORMAL_VEL_CONSTRAINT,NORMAL_ACCEL_CONSTRAINT)
                .build()
        );

        // 11 -> park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), ZOOM_VEL_CONSTRAINT, ZOOM_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(45, -56))
                .build()
        );
        return trajectories;
    }

}
