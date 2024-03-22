package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.centerstage.teamcode.hardware.OPColorSensor;

@Config
public class Intake implements Subsystem {
    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW
    }

    public enum DropdownMode {
        INIT,
        FUNCTIONAL,
    }

    public enum DropdownState {
        DOWN {
            @Override
            public DropdownState previous() {
                return this;
            }
        },
        STACK_2,
        STACK_3,
        STACK_4,
        STACK_5 {
            @Override
            public DropdownState next() {
                return this;
            }
        },
        ALMOST_UP,
        UP,
        MANUAL {
            @Override
            public DropdownState next() {
                return this;
            }
        };

        public DropdownState previous() {
            return values()[ordinal() - 1];
        }

        public DropdownState next() {
            return values()[ordinal() + 1];
        }
    }

    public enum TransferBoxState {
        EMPTY,
        PIXEL1,
        PIXEL2,
        MIDDLE,
        BOTH
    }

    TransferBoxState transfer;

    public IntakeMode intakeMode;
    public DropdownMode dropdownMode;
    public DropdownState dropdownState, lastDropdownState;

    public static double blockedThreshold = 100000;

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.7;
    public static double INTAKE_OUT_SLOW_SPEED = -0.45;
    public static double INTAKE_IN_SLOW_SPEED = 0.225;

    public static double INTAKE_DROPDOWN_UP = 800;
    public static double INTAKE_DROPDOWN_ALMOST_UP = 340;
    public static double INTAKE_DROPDOWN_DOWN = -62; //-170
    public static double INTAKE_DROPDOWN_5 = 120; // min/max 100 / 120
    public static double INTAKE_DROPDOWN_4 = 69; // min/max 69 / 60
    public static double INTAKE_DROPDOWN_3 = 29; // min/max 29 / 0
    public static double INTAKE_DROPDOWN_2 = -10; // min/max -10 / -25
    public static double INTAKE_DDOWN_INITIAL_ANGLE = 160;
    public static double epsilon = 5;
    public double manualPower;

    private OPColorSensor sensor1; // shallow (close to intake)
    private OPColorSensor sensor2; // deep (close to outtake)
    private OPColorSensor sensor3; // horizontal
    private double distance1 = 0.0;
    private double distance2 = 0.0;
    private double distance3 = 0.0;
    public static double sensorThresholdVertical1 = 12.0; // true limit <-> 6.3575844
    public static double sensorThresholdVertical2 = 12.0; // true limit <-> 10.2
    public static double sensorThresholdHorizontal = 21.0; // true *straight in transfer box* <-> 20.0
    public static double sensorThresholdActivate = 60.0; // true *straight in transfer box* <->
    public static boolean intakeSensorsOn = false;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0075, 0.000001, 0.00025);
    private PIDFController pidfController = new PIDFController(pidCoefficients);
    public static double ff = 0.025;
    public static double targetPosition = 0.0;

    private CachingDcMotorEx motor;
    private AxonPlusServo servo;
    private Robot robot;

    public void resetServoPosition() {
        targetPosition = INTAKE_DROPDOWN_DOWN;
        servo.setAbsolutePosition(INTAKE_DROPDOWN_DOWN);
        dropdownMode = DropdownMode.FUNCTIONAL;
        dropdownState = DropdownState.DOWN;
    }

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

    public double getDistance1() {
        return distance1;
    }

    public double getDistance2() {
        return distance2;
    }

    public double getDistance3() {
        return distance3;
    }

    public TransferBoxState getTransferState() {
        return transfer;
    }

    public boolean isPixel1() {
        return (distance1 < sensorThresholdVertical1);
    }

    public boolean isPixel2() {
        return (distance2 < sensorThresholdVertical2);
    }

    public int pixelCount() {
        if (transfer == TransferBoxState.EMPTY) {
            return 0;
        } else if (transfer == TransferBoxState.PIXEL1 ||
                transfer == TransferBoxState.PIXEL2 ||
                transfer == TransferBoxState.MIDDLE) {
            return 1;
        } else if (transfer == TransferBoxState.BOTH) {
            return 2;
        }
        return 0;
    }

    public double getAbsolute() {
        return servo.getAbsolutePosition();
    }

    public double getRelative() {
        return servo.getRelativePosition();
    }

    public double getTargetValue() {
        return targetPosition;
    }

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        servo = new AxonPlusServo(hardwareMap.get(CRServo.class, "intakeServo"),
                hardwareMap.get(AnalogInput.class, "intakeEncoder"));
        sensor1 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor1"));
        sensor2 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor2"));
        sensor3 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor3"));

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        manualPower = 0;
        servo.setDirection(-1);
        servo.setAbsolutePosition(servo.getRelativePosition());
        setPosition(INTAKE_DDOWN_INITIAL_ANGLE);

        transfer = TransferBoxState.EMPTY;

        intakeMode = IntakeMode.IDLE;
        dropdownMode = DropdownMode.INIT;
        dropdownState = lastDropdownState = DropdownState.UP;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;
        servo.update();

        if (intakeMode != IntakeMode.IDLE || intakeSensorsOn) {
            sensor3.update();
            distance3 = sensor3.getDistance();
        } else {
            distance3 = 60.0;
        }
        if (distance3 < sensorThresholdActivate || intakeSensorsOn) {
            sensor1.update();
            distance1 = sensor1.getDistance();
            sensor2.update();
            distance2 = sensor2.getDistance();
        } else {
            distance1 = distance2 = 50;
        }

        if (distance1 > sensorThresholdVertical1 &&
            distance2 > sensorThresholdVertical2) {
            transfer = TransferBoxState.EMPTY;
        } else if (distance1 < sensorThresholdVertical1 &&
            distance2 > sensorThresholdVertical2) {
            transfer = TransferBoxState.PIXEL1;
        } else if (distance1 > sensorThresholdVertical1 &&
            distance2 < sensorThresholdVertical2) {
            transfer = TransferBoxState.PIXEL2;
        } else if (distance1 < sensorThresholdVertical1 &&
            distance2 < sensorThresholdVertical2) {
            if (distance3 < sensorThresholdHorizontal) {
                transfer = TransferBoxState.BOTH;
            } else {
                transfer = TransferBoxState.MIDDLE;
            }
        }

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

        switch (dropdownMode) {
            case INIT:
                pidfController.setTargetPosition(INTAKE_DDOWN_INITIAL_ANGLE);
                servo.setPower(ff + pidfController.update(servo.getAbsolutePosition()));

                if (Math.abs(INTAKE_DDOWN_INITIAL_ANGLE - servo.getRelativePosition()) <= epsilon) {
                    targetPosition = INTAKE_DROPDOWN_UP;
                    servo.setAbsolutePosition(INTAKE_DROPDOWN_UP);
                    dropdownMode = DropdownMode.FUNCTIONAL;
                    dropdownState = DropdownState.UP;
                }

                break;
            case FUNCTIONAL:
                if (dropdownState == DropdownState.MANUAL) {
                    setPower(-manualPower);
                } else {
                    if (lastDropdownState == DropdownState.MANUAL) {
                        manualPower = 0;
                    }

                    switch (dropdownState) {
                        case ALMOST_UP:
                            setPosition(INTAKE_DROPDOWN_ALMOST_UP);
                            break;
                        case UP:
                            setPosition(INTAKE_DROPDOWN_UP);
                            break;
                        case DOWN:
                            setPosition(INTAKE_DROPDOWN_DOWN);
                            break;
                        case STACK_5:
                            setPosition(INTAKE_DROPDOWN_5);
                            break;
                        case STACK_4:
                            setPosition(INTAKE_DROPDOWN_4);
                            break;
                        case STACK_3:
                            setPosition(INTAKE_DROPDOWN_3);
                            break;
                        case STACK_2:
                            setPosition(INTAKE_DROPDOWN_2);
                            break;
                    }

                    pidfController.setTargetPosition(targetPosition);
                    setPower(ff + pidfController.update(servo.getAbsolutePosition()));
                    if (lastDropdownState != dropdownState) {
                        lastDropdownState = dropdownState;
                    }
                }
        }
    }
}
