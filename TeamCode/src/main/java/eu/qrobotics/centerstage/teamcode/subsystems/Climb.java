package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServoImplEx;

@Config
public class Climb implements Subsystem {
    public enum ClimbState {
        PASSIVE,
        ACTIVE,
        CLIMBED,
    }

    // 0.3 - stanga
    // 0.48 - dreapta
    public static double CLIMB_PASSIVE_POSITION = 0.125;
    public static double CLIMB_ACTIVE_POSITION = 0.7;
    public static double offset = 0.38;

    public ClimbState climbState;

    public ElapsedTime timer = new ElapsedTime(0);
    public double timerRestraint = 1.2;

    private static double maxVelocity = 0.08;
    private static double maxAcceleration = 0.015;
    private static double maxJerk = 0.005;
    private MotionProfile motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(CLIMB_PASSIVE_POSITION, 0, 0),
            new MotionState(CLIMB_ACTIVE_POSITION, 0, 0),
            maxVelocity,
            maxAcceleration,
            maxJerk);

    private CachingServoImplEx leftServo;
    private CachingServoImplEx rightServo;

    private Robot robot;

    public double getPosition() {
        return leftServo.getPosition();
    }

    private void setPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position + offset);
    }

    public Climb(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        leftServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbLeft"));
        rightServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbRight"));
        leftServo.setDirection(Servo.Direction.REVERSE);
        climbState = ClimbState.PASSIVE;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        // DEBUG SECTION
//        setPosition(CLIMB_ACTIVE_POSITION);

        switch (climbState) {
            case PASSIVE:
                setPosition(CLIMB_PASSIVE_POSITION);
                break;
            case ACTIVE:
//                setPosition(motionProfile.get(timer.seconds()).getX());
                setPosition(CLIMB_ACTIVE_POSITION);
                break;
            case CLIMBED:
                leftServo.setPwmDisable();
                rightServo.setPwmDisable();
                break;
        }
    }
}
