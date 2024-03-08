package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingCRServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServoImplEx;
import eu.qrobotics.centerstage.teamcode.hardware.OPColorSensor;

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

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -1;
    public static double INTAKE_OUT_SLOW_SPEED = -0.5;
    public static double INTAKE_IN_SLOW_SPEED = 0.225;

    public static double INTAKE_DROPDOWN_UP = 0.38;
    public static double INTAKE_DROPDOWN_DOWN = 0.754;
    public static double INTAKE_DROPDOWN_5 = 0.618;
    public static double INTAKE_DROPDOWN_4 = 0.65;
    public static double INTAKE_DROPDOWN_3 = 0.688;
    public static double INTAKE_DROPDOWN_2 = 0.72;
    public double manualPower;

    private OPColorSensor sensor1; // shallow (close to intake)
    private OPColorSensor sensor2; // deep (close to outtake)
    private boolean isPixel1 = false;
    private boolean isPixel2 = false;
    private double sensorThreshold = 10;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(2, 0, 0);
    private PIDFController pidfController = new PIDFController(pidCoefficients);
    public static double targetPosition = 0.0;

    private CachingDcMotorEx motor;
    private AxonPlusServo servo;
    private Robot robot;

    public void setPosition(double target) {
        targetPosition = target;
    }

    public void setPower(double power) {
        servo.setPower(power);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getVelocity() {
        return motor.getVelocity(AngleUnit.DEGREES);
    }

    public boolean isPixel1() {
        return isPixel1;
    }

    public boolean isPixel2() {
        return isPixel2;
    }

    public int pixelCount() {
        if (isPixel1 && isPixel2) return 2;
        else if (isPixel1 || isPixel2) return 1;
        return 0;
    }

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        servo = new AxonPlusServo(hardwareMap.get(CRServo.class, "intakeServo"),
                hardwareMap.get(AnalogInput.class, "intakeEncoder"));
        sensor1 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor1"));
        sensor2 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor2"));

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setPosition(INTAKE_DROPDOWN_UP);
        manualPower = 0;
        servo.setAbsolutePosition(servo.getRelativePosition());

        intakeMode = IntakeMode.IDLE;
        dropdownState = DropdownState.UP;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        servo.update();

        sensor1.update();

        sensor2.update();

        isPixel1 = (sensor1.getDistance() < sensorThreshold);
        isPixel2 = (sensor2.getDistance() < sensorThreshold);

        switch (intakeMode) {
            case IN:
                motor.setPower(INTAKE_IN_SPEED);
                break;
            case IN_SLOW:
                motor.setPower(INTAKE_IN_SLOW_SPEED);
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

        if (dropdownState == DropdownState.MANUAL) {
            setPower(-manualPower);
        } else {
            if (lastDropdownState == DropdownState.MANUAL) {
                manualPower = 0;
            }

//            switch (dropdownState) {
//                case UP:
//                    setPosition(INTAKE_DROPDOWN_UP);
//                    break;
//                case DOWN:
//                    setPosition(INTAKE_DROPDOWN_DOWN);
//                    break;
//                case STACK_5:
//                    setPosition(INTAKE_DROPDOWN_5);
//                    break;
//                case STACK_4:
//                    setPosition(INTAKE_DROPDOWN_4);
//                    break;
//                case STACK_3:
//                    setPosition(INTAKE_DROPDOWN_3);
//                    break;
//                case STACK_2:
//                    setPosition(INTAKE_DROPDOWN_2);
//                    break;
//            }

//            pidfController.setTargetPosition(targetPosition);
            //servo.setPower(pidfController.update(servo.getAbsolutePosition()));
        }
        lastDropdownState = dropdownState;
    }
}
