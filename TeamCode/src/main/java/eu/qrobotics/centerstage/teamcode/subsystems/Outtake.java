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
    public static double VDIFFY_TRANSFER_POS = 0;
    public static double VDIFFY_SCORE_POS = 0;
    public static double HDIFFY_LEFT_POS = 0;
    public static double HDIFFY_CENTER_POS = 0;
    public static double HDIFFY_RIGHT_POS = 0;
    public static double verticalGain = 0.001;
    public static double horizontalGain = 0.001;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_POS = 0.5;
    public static double FOURBAR_SCORE_POS = 1;
    public static double CLAW_OPEN_POS = 0.25;
    public static double CLAW_CLOSE_POS = 0.55;

    public static double ROTATE_TRANSFER_POS = 0.5;
    public static double ROTATE_LEFT_POS = 1;
    public static double ROTATE_RIGHT_POS = 1;

    // TODO: manual shit
    public double manualFourbarPos;
    public double manualRotatePos;
    public double manualVDiffy;
    public double manualHDiffy;

    public ElapsedTime timer = new ElapsedTime(0);

    public static double transferTimerThreshold = 0.5;
    public static double scoreTimerThreshold = 0.5;

    private CachingServo diffyLeftServo;
    private CachingServo diffyRightServo;
    private CachingServo fourBarServo;
    private CachingServo rotateServo;
    private CachingServo clawServo;

    private Robot robot;

    private double diffyVPosition;
    private double diffyHPosition;
    public static double DIFFYH = 0.0;
    public static double DIFFYV = 0.2;

    public void getManualValues() {
        if (lastOuttakeState == OuttakeState.MANUAL) return;
        manualFourbarPos = fourBarServo.getPosition();
        manualRotatePos = rotateServo.getPosition();
    }

    void updateDiffyPosition() {
        diffyLeftServo.setPosition(diffyVPosition + diffyHPosition);
        diffyRightServo.setPosition(diffyVPosition - diffyHPosition);
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

        diffyVPosition = VDIFFY_TRANSFER_POS;
        diffyHPosition = 0;

        outtakeState = OuttakeState.TRANSFER;
        lastOuttakeState = OuttakeState.TRANSFER;
    }

    public static boolean IS_DISABLED = true;

    @Override
    public void update() {
        if (IS_DISABLED) return;

        // DEBUG SECTION
        clawServo.setPosition(CLAW_CLOSE_POS);
        rotateServo.setPosition(ROTATE_TRANSFER_POS);
        fourBarServo.setPosition(FOURBAR_TRANSFER_POS);

        // 0.2 - pos init in transfer basic
        diffyLeftServo.setPosition(DIFFYV + DIFFYH);
        diffyRightServo.setPosition(DIFFYV - DIFFYH);

        if (lastOuttakeState != outtakeState) {
            timer.reset();
        }

//        switch (outtakeState) {
//            case TRANSFER:
//                fourBarServo.setPosition(FOURBAR_SCORE_POS);
//                rotateState = RotateState.CENTER;
//                rotateServo.setPosition(HDIFFY_CENTER_POS);
//
//                if (timer.seconds() < transferTimerThreshold) {
//                    return;
//                }
//
//                if (getDiffyHorizontal() < HDIFFY_CENTER_POS) {
//                    diffyHPosition = Math.min(diffyHPosition + horizontalGain, HDIFFY_CENTER_POS);
//                } else if (HDIFFY_CENTER_POS < getDiffyHorizontal()) {
//                    diffyHPosition = Math.max(diffyHPosition + horizontalGain, HDIFFY_CENTER_POS);
//                } else {
//                    diffyVPosition = Math.max(diffyVPosition - verticalGain, VDIFFY_TRANSFER_POS);
//                }
//                updateDiffyPosition();
//
//                break;
//            case SCORE:
//                if (getDiffyVertical() < VDIFFY_SCORE_POS) {
//                    diffyVPosition = Math.min(diffyVPosition + verticalGain, VDIFFY_SCORE_POS);
//                }
//                updateDiffyPosition();
//
//                if (timer.seconds() < scoreTimerThreshold) {
//                    return;
//                }
//
//                fourBarServo.setPosition(FOURBAR_SCORE_POS);
//
//                if (getDiffyVertical() == VDIFFY_SCORE_POS) {
//                    switch (rotateState) {
//                        case LEFT:
//                            rotateServo.setPosition(ROTATE_LEFT_POS);
//                            break;
//                        case CENTER:
//                            rotateServo.setPosition(ROTATE_TRANSFER_POS);
//                            break;
//                        case RIGHT:
//                            rotateServo.setPosition(ROTATE_RIGHT_POS);
//                            break;
//                    }
//
//                    switch (diffyHState) {
//                        case LEFT:
//                            diffyHPosition = Math.max(diffyHPosition - horizontalGain, HDIFFY_LEFT_POS);
//                            break;
//                        case CENTER:
//                            if (getDiffyHorizontal() < HDIFFY_CENTER_POS) {
//                                diffyHPosition = Math.max(diffyHPosition + horizontalGain, HDIFFY_CENTER_POS);
//                            }
//                            else if (HDIFFY_CENTER_POS < getDiffyHorizontal()) {
//                                diffyHPosition = Math.max(diffyHPosition - horizontalGain, HDIFFY_CENTER_POS);
//                            }
//                            break;
//                        case RIGHT:
//                            diffyHPosition = Math.max(diffyHPosition + horizontalGain, HDIFFY_RIGHT_POS);
//                            break;
//                    }
//                    updateDiffyPosition();
//                }
//                break;
//            case MANUAL:
//                fourBarServo.setPosition(manualFourbarPos);
//                rotateServo.setPosition(manualRotatePos);
//                updateDiffyPosition();
//                break;
//        }
//
//        switch (clawState) {
//            case OPEN:
//                clawServo.setPosition(CLAW_OPEN_POS);
//                break;
//            case CLOSED:
//                clawServo.setPosition(CLAW_CLOSE_POS);
//                break;
//        }
//        lastOuttakeState = outtakeState;
    }
}
