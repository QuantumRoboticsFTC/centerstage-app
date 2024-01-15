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

public class TrajectoryTestRed {
    public static Pose2d START_POSE = new Pose2d(-37, -65, Math.toRadians(270));

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

    public static List<Trajectory> getTrajectories(int teamProp) {
        List<Trajectory> trajectories = new ArrayList<>();

        // DE DEPARTE :EXCLAMATION:
        // Pose2d(-38.1, -63.5, Math.toRadians(90));
        {
            // TP 1
            {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-43, -43, Math.toRadians(120)))
                        .build()
                );

                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(120), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-40, -51, Math.toRadians(160)))
                        .splineToSplineHeading(new Pose2d(0, -58, Math.toRadians(180)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48, -30, Math.toRadians(180)), Math.toRadians(30))
                        .build()
                );
            }

            // TP 2
            {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-46, -32, Math.toRadians(60)))
                        .build()
                );

                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(120), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-40, -51, Math.toRadians(160)))
                        .splineToSplineHeading(new Pose2d(0, -58, Math.toRadians(180)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(30))
                        .build()
                );
            }

            // TP 3
            {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-33.5, -38, Math.toRadians(20)))
                        .build()
                );

                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(20), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(-40, -51, Math.toRadians(160)))
                        .splineToSplineHeading(new Pose2d(0, -58, Math.toRadians(180)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48, -42.5, Math.toRadians(180)), Math.toRadians(30))
                        .build()
                );
            }

        }

        // DE APROAPE :EXCLAMATION:
        {
            // TP 1
            {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(7, -41, Math.toRadians(140)))
                        .build()
                );

                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(140), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(14, -42, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(48, -30, Math.toRadians(180)), Math.toRadians(30))
                        .build()
                );
            }

            // TP 2
            {
                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(23.5, -28, Math.toRadians(140)))
                        .build()
                );

                trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(140), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                        .lineToSplineHeading(new Pose2d(26, -35, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(30))
                        .build()
                );
            }

            // TP 3
            {
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
        }

        // parked right
        {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -60))
                    .splineToConstantHeading(new Vector2d(62, -60), Math.toRadians(0))
                    .build()
            );
        }
        // parked left
        {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineTo(new Vector2d(50, -13))
                    .build()
            );
        }

        return trajectories;
    }

}