package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.util.InterpLUT;
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
        AUTO_HEIGHT0,
        AUTO_HEIGHT1,
        AUTO_HEIGHT2
    }

    public ElevatorState elevatorState;
    public ElevatorState lastState;
    public TargetHeight targetHeight;

    public static double TRANSFER_POSITION = 0;
    public static double FIRST_POSITION = 250;
    public static double SECOND_POSITION = 500;
    public static double THIRD_POSITION = 830;

    public static double AUTO_HEIGHT0 = 390; // preload
    public static double AUTO_HEIGHT1 = 520; // first cycle
    public static double AUTO_HEIGHT2 = 590; // second cycle
    public static double NEUTRAL_SIDE_THRESHOLD_LOW = 60; // no global coordinates below this point
    public static double diffyHOffset = 230;
    public static double diffyValue = 0;
    public static double IDLE_POWER = 0.13;
    public static double climbedPosition;
    public static double imuPitchGain = 100;

    public double groundPositionOffset;
    public double heightCap = 910;
    public double encoderValue;
    public double manualOffset;

    public double manualPower;

    private ElapsedTime elapsedTime = new ElapsedTime(0);

    public static double lowThreshold = 300.0;
    public static double middleThreshold = 600.0;
    InterpLUT pCoeffs = new InterpLUT();
    double kpLow = 0.0;
    double kpMid = 0.0;
    double kpHigh = 0.0;
    InterpLUT dCoeffs = new InterpLUT();
    double kiLow = 0.0;
    double kiMid = 0.0;
    double kiHigh = 0.0;
    InterpLUT iCoeffs = new InterpLUT();
    double kdLow = 0.0;
    double kdMid = 0.0;
    double kdHigh = 0.0;

    public static PIDCoefficients coefs = new PIDCoefficients(0.0055, 0.000004, 0.00001);
    private PIDFController controller = new PIDFController(coefs);
    public static double ff1 = 0.07;

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
    }

    public void setPower(double power) {
        if (encoderValue > groundPositionOffset + heightCap && power > 0)
            return;
        
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public double getTargetPosition() {
        if (elevatorState == ElevatorState.TRANSFER) {
            return TRANSFER_POSITION + groundPositionOffset + diffyValue;
        }
        switch (targetHeight) {
            case FIRST_LINE:
                return FIRST_POSITION + manualOffset + groundPositionOffset + diffyValue;
            case SECOND_LINE:
                return SECOND_POSITION + manualOffset + groundPositionOffset + diffyValue;
            case THIRD_LINE:
                return THIRD_POSITION + manualOffset + groundPositionOffset + diffyValue;
            case AUTO_HEIGHT0:
                return AUTO_HEIGHT0 + diffyValue;
            case AUTO_HEIGHT1:
                return AUTO_HEIGHT1 + diffyValue;
            case AUTO_HEIGHT2:
                return AUTO_HEIGHT2 + diffyValue;
        }
        return 0;
    }

    public double getTargetPosition(ElevatorState es, TargetHeight ta) {
        if (es == ElevatorState.TRANSFER) {
            return TRANSFER_POSITION + groundPositionOffset + diffyValue;
        }
        switch (ta) {
            case FIRST_LINE:
                return FIRST_POSITION + groundPositionOffset + diffyValue;
            case SECOND_LINE:
                return SECOND_POSITION + groundPositionOffset + diffyValue;
            case THIRD_LINE:
                return THIRD_POSITION + groundPositionOffset + diffyValue;
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
        return encoderValue;
    }

    public Elevator(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderRight"));

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pCoeffs.add(lowThreshold, kpLow);
        pCoeffs.add(middleThreshold, kpMid);
        pCoeffs.add(heightCap, kpHigh);
        pCoeffs.createLUT();

        iCoeffs.add(lowThreshold, kiLow);
        iCoeffs.add(middleThreshold, kiMid);
        iCoeffs.add(heightCap, kiHigh);
        iCoeffs.createLUT();

        dCoeffs.add(lowThreshold, kdLow);
        dCoeffs.add(middleThreshold, kdMid);
        dCoeffs.add(heightCap, kdHigh);
        dCoeffs.createLUT();

        manualOffset = 0;
        groundPositionOffset = -motorLeft.getCurrentPosition();
//        encoderValue = 0;
        encoderValue = -motorLeft.getCurrentPosition();

        elevatorState = lastState = ElevatorState.TRANSFER;
        targetHeight = TargetHeight.FIRST_LINE;

        manualPower = IDLE_POWER;
        climbedPosition = FIRST_POSITION;
        diffyValue = 0;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        encoderValue = -motorLeft.getCurrentPosition();

//        if (elevatorState == ElevatorState.TRANSFER) {
//            manualOffset = 0;
//        }

        diffyValue = 0;
        if (robot.outtake.diffyHState != Outtake.DiffyHorizontalState.CENTER &&
            NEUTRAL_SIDE_THRESHOLD_LOW <= getTargetPosition() &&
            robot.outtake.outtakeState == Outtake.OuttakeState.SCORE) {
            diffyValue = diffyHOffset;
        } else {
            diffyValue = 0;
        }

        PIDCoefficients customCoeffs = new PIDCoefficients(pCoeffs.get(encoderValue + groundPositionOffset),
                                                            iCoeffs.get(encoderValue + groundPositionOffset),
                                                            dCoeffs.get(encoderValue + groundPositionOffset));
//        controller = new PIDFController(customCoeffs);

        if (elevatorState == ElevatorState.LINES ||
            elevatorState == ElevatorState.TRANSFER) {
            controller.setTargetPosition(getTargetPosition());
            setPower(controller.update(getCurrentPosition()) + ff1);
        } else if (elevatorState == ElevatorState.CLIMBED) {
            updateClimbedPosition();
            controller.setTargetPosition(climbedPosition);
            setPower(controller.update(getCurrentPosition()) + ff1);
        } else if (elevatorState == ElevatorState.MANUAL) {
            setPower(manualPower);
        }
    }
}
