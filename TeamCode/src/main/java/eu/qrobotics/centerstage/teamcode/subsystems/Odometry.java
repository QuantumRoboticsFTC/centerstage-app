package eu.qrobotics.centerstage.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.centerstage.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class Odometry extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.692; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 11.04;
    public static double FORWARD_OFFSET = -5.97;

    public static Pose2d LEFT_POSE = new Pose2d(0,  LATERAL_DISTANCE / 2, 0);
    public static Pose2d RIGHT_POSE = new Pose2d(0, -LATERAL_DISTANCE / 2, 0);
    public static Pose2d REAR_POSE = new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90));

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    private Encoder leftEncoder, rightEncoder, rearEncoder;

    public Odometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                LEFT_POSE, // left
                RIGHT_POSE, // right
                REAR_POSE // front
        ));

        // TOOD: astea ar tb schimbate
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
//        rearEncoder.setDirection(Encoder.Direction.REVERSE);
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = rearEncoder.getCurrentPosition();

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) rearEncoder.getCorrectedVelocity();

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}