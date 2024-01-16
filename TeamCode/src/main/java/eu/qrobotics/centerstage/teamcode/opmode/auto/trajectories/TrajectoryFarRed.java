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

public class TrajectoryFarRed {
    public static Pose2d START_POSE = new Pose2d(-38.1, -63.5, Math.toRadians(90));

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

    // teoretic gata
    public static List<Trajectory> centerstageTrajectories(int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-39, -32, Math.toRadians(200)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(200), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-37, -24, Math.toRadians(250)))
                    .splineToSplineHeading(new Pose2d(17, -11, Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(0))
                    .build()
            );
        } else if (teamProp == 2) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-37, -17, Math.toRadians(250)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(250), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-15, -12, Math.toRadians(180)))
                    .splineToSplineHeading(new Pose2d(16, -12, Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(0))
                    .build()
            );
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-34, -28, Math.toRadians(330)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(330), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-37, -24, Math.toRadians(250)))
                    .splineToSplineHeading(new Pose2d(17, -11, Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(0))
                    .build()
            );
        }

        // cycleurile
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(41, -23, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-15, -12, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57, -10, Math.toRadians(160)), Math.toRadians(180))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(160), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-59, -13, Math.toRadians(200)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(200), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-54, -12, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(15, -14, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, -20, Math.toRadians(180)), Math.toRadians(0))
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

    // teoretic gata
    public static List<Trajectory> trussTrajectories(int teamProp, boolean parkedRight) {
        List<Trajectory> trajectories = new ArrayList<>();

        if (teamProp == 1) {
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
        } else if (teamProp == 2) {
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
        } else if (teamProp == 3) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-33.5, -38, Math.toRadians(20)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(20), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-40, -47, Math.toRadians(160)))
                    .splineToSplineHeading(new Pose2d(15, -54, Math.toRadians(180)), Math.toRadians(30))
                    .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(180)), Math.toRadians(30))
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

    public static List<Trajectory> getTrajectories(int teamProp, boolean truss, boolean parkedRight) {
        if (truss) {
            return trussTrajectories(teamProp, parkedRight);
        } else {
            return centerstageTrajectories(teamProp, parkedRight);
        }
    }

}