package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;

@Config
public class Elevator implements Subsystem {
    public enum ElevatorState {
        LINES,
        TRANSFER,
        MANUAL,
        CLIMBED
    }

    public enum TargetHeight {
        FIRST_LINE,
        SECOND_LINE,
        THIRD_LINE,
        AUTO_HEIGHT
    }

    public ElevatorState elevatorState;
    public ElevatorState lastState;
    public TargetHeight targetHeight;

    public static double TRANSFER_POSITION = 0;
    public static double FIRST_POSITION = 250;
    public static double AUTO_HEIGHT = 380;
    public static double SECOND_POSITION = 500;
    public static double THIRD_POSITION = 830;
    public static double IDLE_POWER = 0.13;
    public static double climbedPosition;
    public static double imuPitchGain = 70;

    public double groundPositionOffset;
    public double heightCap = 910;
    public double manualOffset;

    public double manualPower;

    private ElapsedTime elapsedTime = new ElapsedTime(0);

    private static double maxVelocity = 75;
    private static double maxAcceleration = 15;
    private static double maxJerk = 2;
    private MotionProfile mp;
    public static PIDCoefficients coefs = new PIDCoefficients(0.0095, 0.00045, 0.00035);
    private PIDFController controller = new PIDFController(coefs);
    public static double ff1 = 0.05;

    public static double diffyHOffset = 60;

    private CachingDcMotorEx motorLeft;
    private CachingDcMotorEx motorRight;
    private Robot robot;

    public void setElevatorState(ElevatorState es) {
        if (elevatorState != es) {
            lastState = elevatorState;
        }
        elevatorState = es;
    }

    public void setTargetHeight(TargetHeight height) {
        targetHeight = height;

        elapsedTime.reset();

//        mp = MotionProfileGenerator.generateSimpleMotionProfile(
//            new MotionState(getCurrentPosition(), 0, 0),
//            new MotionState(getTargetPosition(height), 0, 0),
//            maxVelocity,
//            maxAcceleration,
//            maxJerk);
    }

    public void setPower(double power) {
        if ((motorRight.getCurrentPosition() > groundPositionOffset + heightCap && power > 0) ||
            (motorRight.getCurrentPosition() < groundPositionOffset && power < 0))
            return;
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public double getTargetPosition() {
        if (elevatorState == ElevatorState.TRANSFER) {
            return TRANSFER_POSITION + groundPositionOffset;
        }
        switch (targetHeight) {
            case FIRST_LINE:
                return FIRST_POSITION + manualOffset + groundPositionOffset;
            case SECOND_LINE:
                if (robot.outtake.diffyHState != Outtake.DiffyHortizontalState.CENTER) {
                    return SECOND_POSITION + manualOffset + groundPositionOffset;
                } else {
                    return SECOND_POSITION + manualOffset + groundPositionOffset + diffyHOffset;
                }
            case THIRD_LINE:
                if (robot.outtake.diffyHState != Outtake.DiffyHortizontalState.CENTER) {
                    return THIRD_POSITION + manualOffset + groundPositionOffset;
                } else {
                    return THIRD_POSITION + manualOffset + groundPositionOffset + diffyHOffset;
                }
            case AUTO_HEIGHT:
                return AUTO_HEIGHT;
        }
        return 0;
    }

    public double getTargetPosition(ElevatorState es, TargetHeight ta) {
        if (es == ElevatorState.TRANSFER) {
            return TRANSFER_POSITION + groundPositionOffset;
        }
        switch (ta) {
            case FIRST_LINE:
                return FIRST_POSITION + groundPositionOffset;
            case SECOND_LINE:
                if (robot.outtake.diffyHState != Outtake.DiffyHortizontalState.CENTER) {
                    return SECOND_POSITION + groundPositionOffset;
                } else {
                    return SECOND_POSITION + groundPositionOffset + diffyHOffset;
                }
            case THIRD_LINE:
                if (robot.outtake.diffyHState != Outtake.DiffyHortizontalState.CENTER) {
                    return THIRD_POSITION + groundPositionOffset;
                } else {
                    return THIRD_POSITION + groundPositionOffset + diffyHOffset;
                }
        }
        return 0;
    }

    private void updateClimbedPosition() {
        climbedPosition = climbedPosition + ((robot.drive.getPitchValue()) * imuPitchGain);
        if (climbedPosition < groundPositionOffset)
            climbedPosition = 0;
        if (climbedPosition > groundPositionOffset + heightCap)
            climbedPosition = groundPositionOffset + heightCap;
        return;
    }

    public double getCurrentPosition() {
        return motorRight.getCurrentPosition();
    }

    public double getCurrentLeft() {
        return motorLeft.getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrentRight() {
        return motorRight.getCurrent(CurrentUnit.AMPS);
    }

    public Elevator(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderRight"));

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        manualOffset = 0;
        groundPositionOffset = -getCurrentPosition();

        elevatorState = lastState = ElevatorState.TRANSFER;
        targetHeight = TargetHeight.FIRST_LINE;

        manualPower = IDLE_POWER;
        climbedPosition = FIRST_POSITION;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

//        if (elevatorState == ElevatorState.TRANSFER) {
//            manualOffset = 0;
//        }

        // TODO: THIS IS EASY **PID TUNER**
        if (elevatorState == ElevatorState.LINES ||
            elevatorState == ElevatorState.TRANSFER) {
            controller.setTargetPosition(getTargetPosition());
            setPower(controller.update(getCurrentPosition()) + ff1);
        } else if (elevatorState == ElevatorState.MANUAL) {
            setPower(manualPower);
        } else if (elevatorState == ElevatorState.CLIMBED) {
            updateClimbedPosition();
            controller.setTargetPosition(climbedPosition);
            setPower(controller.update(getCurrentPosition()) + ff1);
        }

        // TODO: MP IN PID
//        switch (elevatorState) {
//            case AUTOMATIC:
//                updateBasedOnPID();
//                break;
//            case MANUAL:
//                setPower(manualPower);
//                break;
//        }
    }

    private void updateBasedOnPID() {
        MotionState state = mp.get(elapsedTime.seconds());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double power = controller.update(getCurrentPosition()) + ff1;
        setPower(power);
    }
}
