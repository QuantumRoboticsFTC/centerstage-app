package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServoImplEx;

public class Climb implements Subsystem {
    public enum ClimbState {
        PASSIVE,
        ACTIVE,
        CLIMBED,
    }

    public static double CLIMB_PASSIVE_POSITION = 0;
    public static double CLIMB_ACTIVE_POSITION = 1;

    public ClimbState climbState;

    public ElapsedTime timer = new ElapsedTime(0);
    public double timerRestraint = 0.95;

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
        rightServo.setPosition(position);
    }

    public Climb(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        leftServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbLeft"));
        rightServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbRight"));
        rightServo.setDirection(Servo.Direction.REVERSE);

        climbState = ClimbState.PASSIVE;
    }

    @Override
    public void update() {
        // DO STATE ACTION
        switch (climbState) {
            case PASSIVE:
                setPosition(CLIMB_PASSIVE_POSITION);
                break;
            case ACTIVE:
                setPosition(motionProfile.get(timer.seconds()).getX());
                break;
            case CLIMBED:
                leftServo.setPwmDisable();
                rightServo.setPwmDisable();
                break;
        }
    }
}
