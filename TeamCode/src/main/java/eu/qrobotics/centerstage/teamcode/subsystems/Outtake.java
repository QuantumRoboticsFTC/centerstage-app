package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.hardware.OPDistanceSensor;

@Config
public class Outtake implements Subsystem {
    public enum OuttakeState {
        TRANSFER_PREP,
        TRANSFER,
        ABOVE_TRANSFER,
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

    public enum FourBarState {
        TARGET,
        MANUAL
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
    public FourBarState fourBarState;
    public OuttakeState outtakeState;
    public OuttakeState lastOuttakeState;

    // DIFFy
    public static double VDIFFY_ABOVE_TRANSFER_POS = 0.3;
    public static double VDIFFY_TRANSFER_PREP_POS = 0.18; //0.2325
    public static double VDIFFY_TRANSFER_POS = 0.175;
    public static double VDIFFY_SCORE_POS = 0.5175;

    public static double HDIFFY_LEFT_POS = -0.205;
    public static double HDIFFY_CENTER_POS = 0.0425; //-0.095
    public static double HDIFFY_RIGHT_POS = 0.3;

    public static double rotateThresh = 0.5; // rotate thresh
    public static double vDiffyThresholdVS = 0.45; // vertical speed
    public static double vDiffyThresholdH = 0.51; // horizontal moving
    public static double hDiffyToleranceInside = 0.02;
    public static double hDiffyToleranceOutside = 0.27;

    public static double currVDiffy;
    public static double currHDiffy;

    public static double gainVDiffyInsideUp = 0.018;
    public static double gainVDiffyOutsideUp = 0.04;

    public static double gainVDiffyInsideDown = 0.025;
    public static double gainVDiffyOutsideDown = 0.05;
    public static double gainHDiffy = 0.05;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_PREP_POS = 0.13;
    public static double FOURBAR_TRANSFER_POS = 0.13; //0.095
    public static double FOURBAR_ABOVE_TRANSFER_POS = 0.07;
    public static double FOURBAR_SCORE_POS = 0.64; // -0.02
    public static double FOURBAR_SCORE_ANGLED_POS = 0.68;

    public static double CLAW_OPEN_POS = 0.64;
    public static double CLAW_CLOSE_POS = 0.29;

    public static double ROTATE_TRANSFER_POS = 0.383;
    public static double ROTATE_LEFT_POS = 0.115;
    public static double ROTATE_LEFT45_POS = 0.25;
    public static double ROTATE_RIGHT_POS = 0.665;
    public static double ROTATE_RIGHT45_POS = 0.53;
    public static double rotateGain = 1.08; // per ? of hdiffy, rotateGain of rotate

    // TODO: manual stuff
    public double manualFourbarPos;
    public double manualRotatePos;
    public double manualVDiffy;
    public double manualHDiffy;

    public static double yawControlFourbarPos = 0.0;
    public static double yawControlGain = 0.0017;
    public static double sensorRelevanceThresh = 200;
    public static double deltaLimit = 0;

    public ElapsedTime timer = new ElapsedTime(0);

    private CachingServo diffyLeftServo;
    private CachingServo diffyRightServo;
    private CachingServo fourBarServo;
    private CachingServo rotateServo;
    private CachingServo clawServo;
    private OPDistanceSensor sensorUp;
    private OPDistanceSensor sensorDown;

    private Robot robot;

    private double diffyVPosition;
    private double diffyHPosition;
    public double lastVDiffy;
    public double lastHDiffy;

    public double sensorUpCached;
    public double sensorDownCached;

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
                    HDIFFY_CENTER_POS - hDiffyToleranceOutside < currHDiffy) {
                currHDiffy = Math.max(Math.max(currHDiffy - gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS - hDiffyToleranceOutside);
            }
            if (currHDiffy < diffyHPosition &&
                    currHDiffy < HDIFFY_CENTER_POS + hDiffyToleranceOutside) {
                currHDiffy = Math.min(Math.min(currHDiffy + gainHDiffy, diffyHPosition), HDIFFY_CENTER_POS + hDiffyToleranceOutside);
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
                currVDiffy = Math.max(currVDiffy - gainVDiffyInsideDown, diffyVPosition);
            } else {
                currVDiffy = Math.max(currVDiffy - gainVDiffyOutsideDown, diffyVPosition);
            }
        }
        if (currVDiffy <= diffyVPosition) {
            if (currVDiffy <= vDiffyThresholdVS) {
                currVDiffy = Math.min(currVDiffy + gainVDiffyInsideUp, diffyVPosition);
            } else {
                currVDiffy = Math.min(currVDiffy + gainVDiffyOutsideUp, diffyVPosition);
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
        double position = getRotateStatePosition() + rotateGain * (currHDiffy - HDIFFY_CENTER_POS);
        if (outtakeState != OuttakeState.MANUAL) {
            manualRotatePos = 0;
        }
        return position + manualRotatePos;
    }

    private void updateRotatePosition() {
        if (rotateThresh <= currVDiffy) {
            double position = getRotateStatePosition() + rotateGain * (currHDiffy - HDIFFY_CENTER_POS);
            if (outtakeState != OuttakeState.MANUAL) {
                manualRotatePos = 0;
            }
            rotateServo.setPosition(position + manualRotatePos);
        } else {
            rotateServo.setPosition(ROTATE_TRANSFER_POS);
        }
    }

    public double getMeanSensorDistance() {
        return (sensorUpCached + sensorDownCached) * 0.5;
    }

    public double getSensorUp() {
        return sensorUpCached;
    }

    public double getSensorDown() {
        return sensorDownCached;
    }

    public double getDiffyTargetVertical() { return diffyVPosition; }

    public double getDiffyTargetHorizontal() { return diffyHPosition; }

    public double getDiffyTrueVertical() { return currVDiffy; }

    public double getDiffyTrueHorizontal() { return currHDiffy; }

    public Outtake(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;

        diffyLeftServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyLeft"));
        diffyRightServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyRight"));
        fourBarServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeFourBar"));
        rotateServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeRotate"));
        clawServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeClaw"));
        sensorUp = new OPDistanceSensor(hardwareMap.get(DistanceSensor.class, "sensorUp"));
        sensorDown = new OPDistanceSensor(hardwareMap.get(DistanceSensor.class, "sensorDown"));

        diffyLeftServo.setDirection(Servo.Direction.REVERSE);

        lastVDiffy = diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
        lastHDiffy = diffyHPosition = 0;

        fourBarState = FourBarState.TARGET;
        outtakeState = OuttakeState.TRANSFER_PREP;
        lastOuttakeState = OuttakeState.TRANSFER_PREP;
        clawState = ClawState.OPEN;
        rotateState = RotateState.CENTER;
        diffyHState = DiffyHorizontalState.CENTER;

        currVDiffy = VDIFFY_ABOVE_TRANSFER_POS;
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

        sensorUpCached = sensorUp.getDistance();
        sensorDownCached = sensorDown.getDistance();

        if (lastOuttakeState != outtakeState) {
            timer.reset();
        }

        switch (outtakeState) {
            case TRANSFER:
                fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_POS;
                break;
            case TRANSFER_PREP:
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
                if (currVDiffy == diffyVPosition) {
                    fourBarServo.setPosition(FOURBAR_TRANSFER_PREP_POS);
                }
                break;
            case ABOVE_TRANSFER:
                fourBarServo.setPosition(FOURBAR_ABOVE_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_ABOVE_TRANSFER_POS;
                break;
            case SCORE:
                diffyVPosition = VDIFFY_SCORE_POS;
                switch (diffyHState) {
                    case LEFT:
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS;
                        diffyHPosition = HDIFFY_LEFT_POS;
                        break;
                    case CENTER:
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS;
                        diffyHPosition = HDIFFY_CENTER_POS;
                        break;
                    case RIGHT:
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS;
                        diffyHPosition = HDIFFY_RIGHT_POS;
                        break;
                }

                if (currVDiffy == diffyVPosition &&
                        currHDiffy == diffyHPosition &&
                        diffyHState != DiffyHorizontalState.CENTER &&
                        rotateState != RotateState.CENTER &&
                        getMeanSensorDistance() <= sensorRelevanceThresh) {
                    if (sensorUp.getDistance() + deltaLimit < sensorDown.getDistance()) {
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS + (sensorDown.getDistance() - sensorUp.getDistance()) * yawControlGain;
                    } else if (sensorDown.getDistance() + deltaLimit < sensorUp.getDistance()) {
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS - (sensorUp.getDistance() - sensorDown.getDistance()) * yawControlGain;
                    }
                } else {
                    if (diffyHPosition == HDIFFY_CENTER_POS) {
                        yawControlFourbarPos = FOURBAR_SCORE_POS;
                    } else {
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS;
                    }
                }

                if (fourBarState == FourBarState.TARGET) {
                    fourBarServo.setPosition(yawControlFourbarPos);
                } else {
                    fourBarServo.setPosition(manualFourbarPos);
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
