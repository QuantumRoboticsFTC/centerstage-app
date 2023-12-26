package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;

@Config
public class Outtake implements Subsystem {
    public enum OuttakeState {
        TRANSFER_PREP,
        TRANSFER,
        SCORE,
        MANUAL
    }

    public enum RotateState {
        CENTER,
        LEFT,
        RIGHT
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public enum DiffyHortizontalState {
        CENTER,
        LEFT,
        RIGHT
    }

    public ClawState clawState;
    public RotateState rotateState;
    public DiffyHortizontalState diffyHState;
    public OuttakeState outtakeState;
    public OuttakeState lastOuttakeState;

    // TODO: diffy shit
    public static double VDIFFY_TRANSFER_PREP_POS = 0.23; // 0.2275
    public static double VDIFFY_TRANSFER_POS = 0.211; // 0.2085
    public static double VDIFFY_SCORE_POS = 0.59;
    public static double HDIFFY_LEFT_POS = -0.25;
    public static double HDIFFY_CENTER_POS = 0.126;
    public static double HDIFFY_RIGHT_POS = 0.25;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_POS = 0.2;
    public static double FOURBAR_POST_TRANSFER_POS = 0.4;
    public static double FOURBAR_SCORE_POS = 0.65;

    public static double CLAW_OPEN_POS = 0.2;
    public static double CLAW_CLOSE_POS = 0.8;

    public static double ROTATE_TRANSFER_POS = 0.34;
    public static double ROTATE_LEFT_POS = 0.03;
    public static double ROTATE_RIGHT_POS = 0.715;
    public static double rotateGain = 1.3154; // per ? of hdiffy, rotateGain of rotate

    // TODO: manual shit
    public double manualFourbarPos;
    public double manualRotatePos;
    public double manualVDiffy;
    public double manualHDiffy;

    public ElapsedTime timer = new ElapsedTime(0);

    private CachingServo diffyLeftServo;
    private CachingServo diffyRightServo;
    private CachingServo fourBarServo;
    private CachingServo rotateServo;
    private CachingServo clawServo;

    private Robot robot;

    private double diffyVPosition;
    private double diffyHPosition;
    public double lastDiffyVPosition;
    public double lastDiffyHPosition;
    public static double DIFFYH = 0.0;
    public static double DIFFYV = 0.2;

    public void getManualValues() {
        if (lastOuttakeState == OuttakeState.MANUAL) return;
        manualFourbarPos = fourBarServo.getPosition();
        manualHDiffy = diffyHPosition;
        manualVDiffy = diffyVPosition;
        manualRotatePos = 0;
    }

    private boolean updateDiffyPosition() {
        if (outtakeState != OuttakeState.MANUAL) {
            if (Math.min(diffyVPosition - diffyHPosition, diffyVPosition + diffyHPosition) < 0 ||
                    Math.max(diffyVPosition - diffyHPosition, diffyVPosition + diffyHPosition) > 1) {
                diffyVPosition = lastDiffyVPosition;
                diffyHPosition = lastDiffyHPosition;
                return false;
            }
            diffyLeftServo.setPosition(diffyVPosition + diffyHPosition);
            diffyRightServo.setPosition(diffyVPosition - diffyHPosition);
            lastDiffyVPosition = diffyVPosition;
            lastDiffyHPosition = diffyHPosition;
        } else {
            if (Math.min(manualVDiffy - manualHDiffy, manualVDiffy + manualHDiffy) < 0 ||
                    Math.max(manualVDiffy - manualHDiffy, manualVDiffy + manualHDiffy) > 1) {
                manualVDiffy = lastDiffyVPosition;
                manualHDiffy = lastDiffyHPosition;
                return false;
            }
            diffyLeftServo.setPosition(manualVDiffy + manualHDiffy);
            diffyRightServo.setPosition(manualVDiffy - manualHDiffy);
            lastDiffyVPosition = manualVDiffy;
            lastDiffyHPosition = manualHDiffy;
        }
        return true;
    }

    private double getRotateStatePosition() {
        switch (rotateState) {
            case LEFT:
                return ROTATE_LEFT_POS;
            case CENTER:
                return ROTATE_TRANSFER_POS;
            case RIGHT:
                return ROTATE_RIGHT_POS;
        }
        return 0;
    }

    public double getRotatePosition() {
        double position = getRotateStatePosition() + rotateGain * (diffyHPosition - HDIFFY_CENTER_POS);
        if (outtakeState != OuttakeState.MANUAL) {
            manualRotatePos = 0;
        }
        return position + manualRotatePos;
    }

    private void updateRotatePosition() {
        double position = getRotateStatePosition() + rotateGain * (diffyHPosition - HDIFFY_CENTER_POS);
        if (outtakeState != OuttakeState.MANUAL) {
            manualRotatePos = 0;
        }

        rotateServo.setPosition(position + manualRotatePos);
    }

    public double getDiffyVertical() {
        return diffyVPosition;
    }

    public double getDiffyHorizontal() {
        return diffyHPosition;
    }

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        diffyLeftServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyLeft"));
        diffyRightServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyRight"));
        fourBarServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeFourBar"));
        rotateServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeRotate"));
        clawServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeClaw"));

        diffyLeftServo.setDirection(Servo.Direction.REVERSE);

        lastDiffyVPosition = diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
        lastDiffyHPosition = diffyHPosition = 0;

        outtakeState = OuttakeState.TRANSFER_PREP;
        lastOuttakeState = OuttakeState.TRANSFER_PREP;
        clawState = ClawState.OPEN;
        rotateState = RotateState.CENTER;
        diffyHState = DiffyHortizontalState.CENTER;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        // DEBUG SECTION
//        clawServo.setPosition(CLAW_CLOSE_POS);
//        rotateServo.setPosition(ROTATE_TRANSFER_POS);
//        fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
//
//        diffyLeftServo.setPosition(DIFFYV + DIFFYH);
//        diffyRightServo.setPosition(DIFFYV - DIFFYH);

        if (lastOuttakeState != outtakeState) {
            timer.reset();
        }

        switch (outtakeState) {
            case TRANSFER_PREP:
                if (0.05 < timer.seconds() &&
                    timer.seconds() < 0.15) {
                    fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                }

                if (getDiffyHorizontal() != HDIFFY_CENTER_POS &&
                    0.25 < timer.seconds()) {
                    diffyHPosition = HDIFFY_CENTER_POS;
                }

                if (getDiffyHorizontal() == HDIFFY_CENTER_POS &&
                    0.45 < timer.seconds()) {
                    fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                    diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
                }
                break;
            case TRANSFER:
                if (0.05 < timer.seconds() &&
                    timer.seconds() < 0.15) {
                    fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                }

                if (getDiffyHorizontal() != HDIFFY_CENTER_POS &&
                    0.25 < timer.seconds()) {
                    diffyHPosition = HDIFFY_CENTER_POS;
                }

                if (getDiffyHorizontal() == HDIFFY_CENTER_POS &&
                    0.45 < timer.seconds()) {
                    diffyVPosition = VDIFFY_TRANSFER_POS;
                }

                if (0.65 < timer.seconds()) {
                    fourBarServo.setPosition(FOURBAR_POST_TRANSFER_POS);
                }
                break;
            case SCORE:
                if (0.05 < timer.seconds() &&
                    timer.seconds() < 0.15) {
                    fourBarServo.setPosition(FOURBAR_POST_TRANSFER_POS);
                }

                if (getDiffyVertical() < VDIFFY_SCORE_POS &&
                    0.35 < timer.seconds()) {
                    diffyVPosition = VDIFFY_SCORE_POS;
                }

                if (getDiffyVertical() == VDIFFY_SCORE_POS &&
                    0.65 < timer.seconds()) {
                    fourBarServo.setPosition(FOURBAR_SCORE_POS);
                    switch (diffyHState) {
                        case LEFT:
                            diffyHPosition = HDIFFY_LEFT_POS;
                            break;
                        case CENTER:
                            diffyHPosition = HDIFFY_CENTER_POS;
                            break;
                        case RIGHT:
                            diffyHPosition = HDIFFY_RIGHT_POS;
                            break;
                    }
                }
                break;
            case MANUAL:
                fourBarServo.setPosition(manualFourbarPos);
                break;
        }

        updateDiffyPosition();
        updateRotatePosition();

        switch (clawState) {
            case OPEN:
                clawServo.setPosition(CLAW_OPEN_POS);
                break;
            case CLOSED:
                clawServo.setPosition(CLAW_CLOSE_POS);
                break;
        }
        lastOuttakeState = outtakeState;
    }
}
