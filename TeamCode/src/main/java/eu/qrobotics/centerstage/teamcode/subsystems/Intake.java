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
        IDLE,
        OUT
    }

    public enum DropdownState {
        DOWN,
        UP,
        MANUAL
    }

    public IntakeMode intakeMode;
    public DropdownState dropdownState, lastDropdownState;

    public static double blockedThreshold = 100000;

    public static double INTAKE_IN_SPEED = -1;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = 1;

    public static double INTAKE_DROPDOWN_UP = 0.5;
    public static double INTAKE_DROPDOWN_DOWN = 0.7;
    public double manualPosition;

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
            case IDLE:
                motor.setPower(INTAKE_IDLE_SPEED);
                break;
            case OUT:
                motor.setPower(INTAKE_OUT_SPEED);
                break;
        }

        switch (dropdownState) {
            case UP:
                servo.setPosition(INTAKE_DROPDOWN_UP);
                break;
            case DOWN:
                servo.setPosition(INTAKE_DROPDOWN_DOWN);
                break;
            case MANUAL:
                servo.setPosition(manualPosition);
                break;
        }
        lastDropdownState = dropdownState;
    }
}
