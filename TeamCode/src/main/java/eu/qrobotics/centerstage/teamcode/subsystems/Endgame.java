package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServoImplEx;

@Config
public class Endgame implements Subsystem {
    public enum ClimbState {
        PASSIVE,
        SHOOTER,
        ACTIVE,
        CLIMBED,
    }

    public enum DroneState {
        PASSIVE,
        ACTIVE
    }

    public ClimbState climbState;
    public DroneState droneState;

    public ElapsedTime subsystemTimer = new ElapsedTime(0);

    public static double CLIMB_PASSIVE_POSITION = 0;
    public static double CLIMB_SHOOTER_POSITION = 0.4;
    public static double CLIMB_ACTIVE_POSITION = 0.6;
    public static double offset = 0.09;
    public static double shooterClimbPosition;

    public static double SHOOTER_PASSIVE_POSITION = 0.9;
    public static double SHOOTER_ACTIVE_POSITION = 0.4;

    private CachingServoImplEx leftServo;
    private CachingServoImplEx rightServo;
    private CachingServo droneServo;

    private Robot robot;

    private void updateShooterPosition() {
//        TODO: dynamic shooter

        return;
    }

    public double getPosition() {
        return leftServo.getPosition();
    }

    private void setPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position + offset);
    }

    public void disableClimber() {
        leftServo.setPwmDisable();
        rightServo.setPwmDisable();
    }

    public void enableClimber() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
    }

    public Endgame(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        leftServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbLeft"));
        rightServo = new CachingServoImplEx(hardwareMap.get(ServoImplEx.class, "climbRight"));
        droneServo = new CachingServo(hardwareMap.get(Servo.class, "droneServo"));

        rightServo.setDirection(Servo.Direction.REVERSE);
        subsystemTimer.reset();

        climbState = ClimbState.PASSIVE;
        droneState = DroneState.PASSIVE;
        shooterClimbPosition = CLIMB_SHOOTER_POSITION;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        // DEBUG SECTION
//        setPosition(CLIMB_ACTIVE_POSITION);

        if (subsystemTimer.seconds() < 0.3) {
            disableClimber();
        }

        updateShooterPosition();

        switch (climbState) {
            case PASSIVE:
                setPosition(CLIMB_PASSIVE_POSITION);
                break;
            case SHOOTER:
                setPosition(CLIMB_SHOOTER_POSITION);
                break;
            case ACTIVE:
                setPosition(CLIMB_ACTIVE_POSITION);
                break;
            case CLIMBED:
//                leftServo.setPwmDisable();
//                rightServo.setPwmDisable();
                break;
        }

        switch (droneState) {
            case PASSIVE:
                droneServo.setPosition(SHOOTER_PASSIVE_POSITION);
                break;
            case ACTIVE:
                droneServo.setPosition(SHOOTER_ACTIVE_POSITION);
                break;
        }
    }
}
