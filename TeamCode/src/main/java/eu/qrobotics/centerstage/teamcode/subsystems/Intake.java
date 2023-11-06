package eu.qrobotics.centerstage.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {
    public enum IntakeMode {
        IN,
        IDLE,
        OUT
    }

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -1;

    private IntakeMode intakeMode;

    private Robot robot;

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        //motor servo init

        intakeMode = IntakeMode.IDLE;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        switch (intakeMode) {
            case IN:
                ;
                break;
            case IDLE:
                ;
                break;
            case OUT:
                ;
                break;
        }
    }
}
