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
        POST_TRANSFER,
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
    public static double VDIFFY_TRANSFER_PREP_POS = 0.25;
    public static double VDIFFY_TRANSFER_POS = 0.215;
    public static double VDIFFY_SCORE_POS = 0.56;
    public static double HDIFFY_LEFT_POS = -0.14;
    public static double HDIFFY_CENTER_POS = 0.115;
    public static double HDIFFY_RIGHT_POS = 0.38;
    public static double vDiffyThreshold = 0.54;
    public static double currVDiffy;
    public static double currHDiffy;
    public static double gainVDiffyUp = 0.03;
    public static double gainVDiffyDown = 0.0125;
    public static double gainHDiffy = 0.03;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_POS = 0.13;
    public static double FOURBAR_POST_TRANSFER_POS = 0.16;
    public static double FOURBAR_SCORE_POS = 0.65;

    public static double CLAW_OPEN_POS = 0.2;
    public static double CLAW_CLOSE_POS = 0.8;

    public static double ROTATE_TRANSFER_POS = 0.345;
    public static double ROTATE_LEFT_POS = 0.03;
    public static double ROTATE_RIGHT_POS = 0.68;
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
    public double lastVDiffy;
    public double lastHDiffy;
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
        if (vDiffyThreshold < currVDiffy) {
            if (diffyHPosition < currHDiffy) {
                currHDiffy = Math.max(currHDiffy - gainHDiffy, diffyHPosition);
            }
            if (currHDiffy < diffyHPosition) {
                currHDiffy = Math.min(currHDiffy + gainHDiffy, diffyHPosition);
            }
        }
        if (diffyVPosition < currVDiffy &&
            currHDiffy == diffyHPosition) {
            currVDiffy = Math.max(currVDiffy - gainVDiffyDown, diffyVPosition);
        }
        if (currVDiffy < diffyVPosition) {
            currVDiffy = Math.min(currVDiffy + gainVDiffyUp, diffyVPosition);
        }


        if (outtakeState != OuttakeState.MANUAL) {
            if (Math.min(currVDiffy - currHDiffy, currVDiffy + currHDiffy) < 0 ||
                    Math.max(currVDiffy - currHDiffy, currVDiffy + currHDiffy) > 1) {
                diffyVPosition = lastVDiffy;
                diffyHPosition = lastHDiffy;
                return false;
            }
            diffyLeftServo.setPosition(currVDiffy + currHDiffy);
            diffyRightServo.setPosition(currVDiffy - currHDiffy);
            lastVDiffy = currVDiffy;
            lastHDiffy = currHDiffy;
        } else {
            if (Math.min(manualVDiffy - manualHDiffy, manualVDiffy + manualHDiffy) < 0 ||
                    Math.max(manualVDiffy - manualHDiffy, manualVDiffy + manualHDiffy) > 1) {
                manualVDiffy = lastVDiffy;
                manualHDiffy = lastHDiffy;
                return false;
            }
            diffyLeftServo.setPosition(manualVDiffy + manualHDiffy);
            diffyRightServo.setPosition(manualVDiffy - manualHDiffy);
            lastVDiffy = manualVDiffy;
            lastHDiffy = manualHDiffy;
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

        lastVDiffy = diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
        lastHDiffy = diffyHPosition = 0;

        outtakeState = OuttakeState.TRANSFER_PREP;
        lastOuttakeState = OuttakeState.TRANSFER_PREP;
        clawState = ClawState.OPEN;
        rotateState = RotateState.CENTER;
        diffyHState = DiffyHortizontalState.CENTER;

        currVDiffy = VDIFFY_TRANSFER_PREP_POS;
        currHDiffy = HDIFFY_CENTER_POS;
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
                clawState = ClawState.OPEN;
                fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
                break;
            case TRANSFER:
                fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_POS;
                break;
            case POST_TRANSFER:
                fourBarServo.setPosition(FOURBAR_POST_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
                break;
            case SCORE:
                fourBarServo.setPosition(FOURBAR_SCORE_POS);
                diffyVPosition = VDIFFY_SCORE_POS;
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
