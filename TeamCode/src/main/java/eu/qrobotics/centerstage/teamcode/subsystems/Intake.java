package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.CachingCRServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServoImplEx;

@Config
public class Intake implements Subsystem {
    public enum IntakeMode {
        IN,
        IN_SLOW,
        IN_QUICK,
        IDLE,
        OUT,
        OUT_SLOW
    }

    public enum DropdownState {
        DOWN,
        UP,
        STACK_5,
        STACK_4,
        STACK_3,
        STACK_2,
        MANUAL
    }

    public IntakeMode intakeMode;
    public DropdownState dropdownState, lastDropdownState;

    public static double blockedThreshold = 100000;

    public static double INTAKE_IN_QUICK_SPEED = -1;
    public static double INTAKE_IN_SPEED = -0.8;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = 1;
    public static double INTAKE_OUT_SLOW_SPEED = 0.3;
    public static double INTAKE_IN_SLOW_SPEED = -0.225;

    public static double INTAKE_DROPDOWN_UP = 0.38;
    public static double INTAKE_DROPDOWN_DOWN = 0.754;
    public static double INTAKE_DROPDOWN_5 = 0.618;
    public static double INTAKE_DROPDOWN_4 = 0.647;
    public static double INTAKE_DROPDOWN_3 = 0.688;
    public static double INTAKE_DROPDOWN_2 = 0.72;
    public static double manualPosition;

    private CachingDcMotorEx motor;
    private CachingServo servo;
    private Robot robot;

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getVelocity() {
        return motor.getVelocity(AngleUnit.DEGREES);
    }

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        servo = new CachingServo(hardwareMap.get(Servo.class, "intakeServo"));

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo.setPosition(INTAKE_DROPDOWN_UP);

        intakeMode = IntakeMode.IDLE;
        dropdownState = DropdownState.UP;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        switch (intakeMode) {
            case IN:
                motor.setPower(INTAKE_IN_SPEED);
                break;
            case IN_SLOW:
                motor.setPower(INTAKE_IN_SLOW_SPEED);
                break;
            case IN_QUICK:
                motor.setPower(INTAKE_IN_QUICK_SPEED);
                break;
            case IDLE:
                motor.setPower(INTAKE_IDLE_SPEED);
                break;
            case OUT:
                motor.setPower(INTAKE_OUT_SPEED);
                break;
            case OUT_SLOW:
                motor.setPower(INTAKE_OUT_SLOW_SPEED);
                break;
        }

        switch (dropdownState) {
            case UP:
                servo.setPosition(INTAKE_DROPDOWN_UP);
                break;
            case DOWN:
                servo.setPosition(INTAKE_DROPDOWN_DOWN);
                break;
            case STACK_5:
                servo.setPosition(INTAKE_DROPDOWN_5);
                break;
            case STACK_4:
                servo.setPosition(INTAKE_DROPDOWN_4);
                break;
            case STACK_3:
                servo.setPosition(INTAKE_DROPDOWN_3);
                break;
            case STACK_2:
                servo.setPosition(INTAKE_DROPDOWN_2);
                break;
            case MANUAL:
                servo.setPosition(manualPosition);
                break;
        }
        lastDropdownState = dropdownState;
    }
}
