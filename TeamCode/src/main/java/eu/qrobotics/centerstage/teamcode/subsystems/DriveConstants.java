package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

@Config
public class DriveConstants {
        public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.5, 0, 0.5);
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8,0,0.2);
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0.75);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0.2);

    public static double LATERAL_MULTIPLIER = 1.85853;

    public static double WHEEL_RADIUS = 3.780 / 2.0; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 12.57; // in

    public static double MAX_ANG_VEL = Math.toRadians(270);
    public static double MAX_ANG_ACCEL = Math.toRadians(270);
    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 40;

    public static TrajectoryVelocityConstraint ZOOM_VEL_CONSTRAINT = new MecanumVelocityConstraint(80, TRACK_WIDTH, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint ZOOM_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(70);

    public static TrajectoryVelocityConstraint BASE_VEL_CONSTRAINT = new MecanumVelocityConstraint(60, TRACK_WIDTH, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint BASE_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(50);

    public static TrajectoryVelocityConstraint NORMAL_VEL_CONSTRAINT = new MecanumVelocityConstraint(40, TRACK_WIDTH, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint NORMAL_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(40);

    public static TrajectoryVelocityConstraint SLOW_VEL_CONSTRAINT = new MecanumVelocityConstraint(7, TRACK_WIDTH, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint SLOW_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(30);

    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double kV = 0.01225;
    public static double kA = 0.00415;
    public static double kStatic = 0.055;

    public static Pose2d cameraPose = new Pose2d(8.4, 0, Math.toRadians(0));

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}