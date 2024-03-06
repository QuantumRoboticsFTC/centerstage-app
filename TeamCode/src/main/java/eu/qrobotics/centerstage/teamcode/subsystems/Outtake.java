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
        LEFT45,
        RIGHT,
        RIGHT45
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public enum DiffyHorizontalState {
        CENTER,
        LEFT,
        RIGHT
    }

    public ClawState clawState;
    public RotateState rotateState;
    public DiffyHorizontalState diffyHState;
    public OuttakeState outtakeState;
    public OuttakeState lastOuttakeState;

    // DIFFy
    public static double VDIFFY_OFFSET=-0.00329;
    public static double VDIFFY_TRANSFER_PREP_POS = 0.245+VDIFFY_OFFSET;
    public static double VDIFFY_TRANSFER_POS = 0.21+VDIFFY_OFFSET; //0.215
    public static double VDIFFY_SCORE_POS = 0.56+VDIFFY_OFFSET;

    public static double HDIFFY_OFFSET=-0.0945;
    public static double HDIFFY_LEFT_POS = -0.13+HDIFFY_OFFSET;
    public static double HDIFFY_CENTER_POS = 0.115+HDIFFY_OFFSET;
    public static double HDIFFY_RIGHT_POS = 0.36+HDIFFY_OFFSET;

    public static double vDiffyThresholdVS = 0.4; // vertical speed
    public static double vDiffyThresholdH = 0.55; // horizontal moving
    public static double hDiffyToleranceInside = 0.02;
    public static double hDiffyToleranceOutside = 0.27;

    public static double currVDiffy;
    public static double currHDiffy;

    public static double gainVDiffyInside = 0.01;
    public static double gainVDiffyOutside = 0.02;
    public static double gainHDiffy = 0.013;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_POS = 0.115; //0.13
    public static double FOURBAR_POST_TRANSFER_POS = 0.16;
    public static double FOURBAR_SCORE_POS = 0.58;
    public static double FOURBAR_SCORE_ANGLED_POS = 0.63;

    public static double CLAW_OPEN_POS = 0.2;
    public static double CLAW_CLOSE_POS = 0.85;

    public static double ROTATE_TRANSFER_POS =0.33;
    public static double ROTATE_LEFT_POS = 0.665+0.015;
    public static double ROTATE_LEFT45_POS = 0.495+0.015;
    public static double ROTATE_RIGHT_POS = 0+0.015;
    public static double ROTATE_RIGHT45_POS = 0.165+0.015;
    public static double rotateGain = 1.3154; // per ? of hdiffy, rotateGain of rotate

    // TODO: manual stuff
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

    public void getManualValues() {
        if (lastOuttakeState == OuttakeState.MANUAL) return;
        manualFourbarPos = fourBarServo.getPosition();
        manualHDiffy = diffyHPosition;
        manualVDiffy = diffyVPosition;
        manualRotatePos = 0;
    }

    private boolean updateDiffyPosition() {
        // MANUAL update
        if (outtakeState == OuttakeState.MANUAL) {
            diffyVPosition = manualVDiffy;
            if (diffyVPosition < vDiffyThresholdH) {
                if (HDIFFY_CENTER_POS < manualHDiffy) {
                    diffyHPosition = Math.min(manualHDiffy, HDIFFY_CENTER_POS + hDiffyToleranceInside);
                } else {
                    diffyHPosition = Math.max(manualHDiffy, HDIFFY_CENTER_POS - hDiffyToleranceInside);
                }
            } else {
                if (HDIFFY_CENTER_POS < manualHDiffy) {
                    diffyHPosition = Math.min(manualHDiffy, HDIFFY_CENTER_POS + hDiffyToleranceOutside);
                } else {
                    diffyHPosition = Math.max(manualHDiffy, HDIFFY_CENTER_POS - hDiffyToleranceOutside);
                }
            }
        }

        // MOVE to target position
        if (vDiffyThresholdH < currVDiffy) {
            if (diffyHPosition < currHDiffy &&
                HDIFFY_CENTER_POS + hDiffyToleranceOutside < currHDiffy) {
                currHDiffy = Math.max(Math.max(currHDiffy - gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS + hDiffyToleranceOutside);
            }
            if (currHDiffy < diffyHPosition &&
                currHDiffy < HDIFFY_CENTER_POS + hDiffyToleranceOutside) {
                currHDiffy = Math.min(Math.min(currHDiffy + gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS - hDiffyToleranceOutside);
            }
        } else {
            if (diffyHPosition < currHDiffy &&
                HDIFFY_CENTER_POS + hDiffyToleranceInside < currHDiffy) {
                currHDiffy = Math.max(Math.max(currHDiffy - gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS + hDiffyToleranceInside);
            }
            if (currHDiffy < diffyHPosition &&
                currHDiffy < HDIFFY_CENTER_POS + hDiffyToleranceInside) {
                currHDiffy = Math.min(Math.min(currHDiffy + gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS - hDiffyToleranceInside);
            }
        }
        if (diffyVPosition < currVDiffy &&
            currHDiffy == diffyHPosition) {
            if (currVDiffy <= vDiffyThresholdVS) {
                currVDiffy = Math.max(currVDiffy - gainVDiffyInside, diffyVPosition);
            } else {
                currVDiffy = Math.max(currVDiffy - gainVDiffyOutside, diffyVPosition);
            }
        }
        if (currVDiffy <= diffyVPosition) {
            if (currVDiffy <= vDiffyThresholdVS) {
                currVDiffy = Math.min(currVDiffy + gainVDiffyInside, diffyVPosition);
            } else {
                currVDiffy = Math.min(currVDiffy + gainVDiffyOutside, diffyVPosition);
            }
        }

        // Do NOT break servo
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
        return true;
    }

    private double getRotateStatePosition() {
        switch (rotateState) {
            case LEFT:
                return ROTATE_LEFT_POS;
            case LEFT45:
                return ROTATE_LEFT45_POS;
            case CENTER:
                return ROTATE_TRANSFER_POS;
            case RIGHT:
                return ROTATE_RIGHT_POS;
            case RIGHT45:
                return ROTATE_RIGHT45_POS;
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

    public double getDiffyVertical() { return diffyVPosition; }

    public double getDiffyHorizontal() { return diffyHPosition; }

    public Outtake(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
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
        diffyHState = DiffyHorizontalState.CENTER;

        currVDiffy = VDIFFY_TRANSFER_PREP_POS;
        currHDiffy = HDIFFY_CENTER_POS;

        if (isAutonomous) {
            gainVDiffyInside = 0.01;
            gainVDiffyOutside = 0.02;
            gainHDiffy = 0.013;
        } else {
            gainVDiffyInside = 0.0125;
            gainVDiffyOutside = 0.025;
            gainHDiffy = 0.03;
        }
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
                diffyVPosition = VDIFFY_SCORE_POS;
                switch (diffyHState) {
                    case LEFT:
                        fourBarServo.setPosition(FOURBAR_SCORE_ANGLED_POS);
                        diffyHPosition = HDIFFY_LEFT_POS;
                        break;
                    case CENTER:
                        fourBarServo.setPosition(FOURBAR_SCORE_POS);
                        diffyHPosition = HDIFFY_CENTER_POS;
                        break;
                    case RIGHT:
                        fourBarServo.setPosition(FOURBAR_SCORE_ANGLED_POS);
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
