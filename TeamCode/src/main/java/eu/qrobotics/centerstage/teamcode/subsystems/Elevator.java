package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;

@Config
public class Elevator implements Subsystem {
    public enum ElevatorState {
        AUTOMATIC,
        MANUAL
    }

    public enum TargetHeight {
        TRANSFER,
        SCORE
    }

    public ElevatorState elevatorState;
    public ElevatorState lastState;
    public TargetHeight targetHeight;

    public static double TRANSFER_POSITION = 10;
    public static double SCORE_POSITION = 50;
    public static double DOWN_POWER = -0.75;
    public static double UP_POWER = -0.75;
    public static double IDLE_POWER = 0.18;

    public static double tolerance;
    public double manualOffset;

    public double manualPower;

    private ElapsedTime elapsedTime = new ElapsedTime(0);

    private static double maxVelocity = 75;
    private static double maxAcceleration = 15;
    private static double maxJerk = 2;
    private MotionProfile mp;
    public static PIDCoefficients coefs = new PIDCoefficients(0, 0, 0);
    private PIDFController controller = new PIDFController(coefs);
    private double ff1 = 0, ff2 = 0;

    private CachingDcMotorEx motorLeft;
    private CachingDcMotorEx motorRight;
    private Robot robot;

    public void setTargetHeight(TargetHeight height) {
        targetHeight = height;

        elapsedTime.reset();

        mp = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(getCurrentPosition(), 0, 0),
            new MotionState(getTargetPosition(height), 0, 0),
            maxVelocity,
            maxAcceleration,
            maxJerk);
    }

    public void setPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public double getTargetPosition() {
        switch (targetHeight) {
            case TRANSFER:
                return TRANSFER_POSITION + manualOffset;
            case SCORE:
                return SCORE_POSITION + manualOffset;
        }
        return 0;
    }

    public double getTargetPosition(TargetHeight ta) {
        switch (ta) {
            case TRANSFER:
                return TRANSFER_POSITION;
            case SCORE:
                return SCORE_POSITION;
        }
        return 0;
    }

    public double getCurrentPosition() {
        return motorLeft.getCurrentPosition();
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

        tolerance = 75;
        manualOffset = 0;

        elevatorState = lastState = ElevatorState.AUTOMATIC;
        targetHeight = TargetHeight.TRANSFER;

        manualPower = IDLE_POWER;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        if (elevatorState == ElevatorState.MANUAL) {
            setPower(manualPower);
        } else {
            setPower(IDLE_POWER);
        }

        if (targetHeight == TargetHeight.TRANSFER) {
            manualOffset = 0;
        }

//        switch (elevatorState) {
//            case AUTOMATIC:
//                switch (targetHeight) {
//                    case TRANSFER:
//                        if (getCurrentPosition() >= getTargetPosition() + tolerance) {
//                            setPower(DOWN_POWER);
//                        }
//                        break;
//                    case SCORE:
//                        if (getCurrentPosition() + tolerance <= getTargetPosition()) {
//                            setPower(UP_POWER);
//                        }
//                        break;
//                }
//                break;
//            case MANUAL:
//                setPower(manualPower);
//                break;
//        }

        // TODO: THIS IS EASY **PID TUNER**
//        switch (elevatorState) {
//            case AUTOMATIC:
//                controller.setTargetPosition(getTargetPosition());
//                setPower(controller.update(getCurrentPosition()) + ff1 + getCurrentPosition() * ff2);
//                break;
//            case MANUAL:
//                setPower(manualPower);
//                break;
//        }

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

        double power = controller.update(getCurrentPosition()) + ff1 + getCurrentPosition() * ff2;
        setPower(power);
    }
}
