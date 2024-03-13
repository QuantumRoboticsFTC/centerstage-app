package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.hardware.OPDistanceSensor;

@Config
public class Outtake_CR implements Subsystem {
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
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);
    private PIDFController pidfControllerLeft = new PIDFController(pidCoefficients);
    private PIDFController pidfControllerRight = new PIDFController(pidCoefficients);
    public static double ff = 0.05;

    public static double VDIFFY_ABOVE_TRANSFER_POS = 0.35;
    public static double VDIFFY_TRANSFER_PREP_POS = 0.2275;
    public static double VDIFFY_TRANSFER_POS = 0.222;
    public static double VDIFFY_SCORE_POS = 0.556;

    public static double HDIFFY_LEFT_POS = -0.224;
    public static double HDIFFY_CENTER_POS = 0.02;
    public static double HDIFFY_RIGHT_POS = 0.265;

    public static double rotateThresh = 0.52; // rotate thresh
    public static double vDiffyThresholdVS = 0.45; // vertical speed
    public static double vDiffyThresholdH = 0.55; // horizontal moving
    public static double hDiffyToleranceInside = 0.02;
    public static double hDiffyToleranceOutside = 0.27;

    public static double currVDiffy;
    public static double currHDiffy;

    public static double gainVDiffyInside = 0.02;
    public static double gainVDiffyOutside = 0.04;
    public static double gainHDiffy = 0.0275;

    // Other Outtake Servo Values
    public static double FOURBAR_TRANSFER_PREP_POS = 0.13;
    public static double FOURBAR_TRANSFER_POS = 0.115;
    public static double FOURBAR_ABOVE_TRANSFER_POS = 0.09;
    public static double FOURBAR_SCORE_POS = 0.66;
    public static double FOURBAR_SCORE_ANGLED_POS = 0.68;

    public static double CLAW_OPEN_POS = 1;
    public static double CLAW_CLOSE_POS = 0.4;

    public static double ROTATE_TRANSFER_POS = 0.383;
    public static double ROTATE_LEFT_POS = 0.115;
    public static double ROTATE_LEFT45_POS = 0.25;
    public static double ROTATE_RIGHT_POS = 0.665;
    public static double ROTATE_RIGHT45_POS = 0.53;
    public static double rotateGain = 1.16; // per ? of hdiffy, rotateGain of rotate

    // TODO: manual stuff
    public double manualFourbarPos;
    public double manualRotatePos;
    public double manualVDiffy;
    public double manualHDiffy;

    public double yawControlFourbarPos = 0.0;
    public double yawControlGain = 0.25;

    public ElapsedTime timer = new ElapsedTime(0);

    private AxonPlusServo diffyLeftServo;
    private AxonPlusServo diffyRightServo;
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
        pidfControllerLeft.setTargetPosition((currVDiffy + currHDiffy)*360);
        diffyLeftServo.setPower(ff + pidfControllerLeft.update(diffyLeftServo.getAbsolutePosition()));
        pidfControllerLeft.setTargetPosition((currVDiffy - currHDiffy) * 360);
        diffyRightServo.setPower(ff + pidfControllerRight.update(diffyRightServo.getAbsolutePosition()));
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
        return (sensorUp.getDistance() + sensorDown.getDistance()) * 0.5;
    }

    public double getDiffyTargetVertical() { return diffyVPosition; }

    public double getDiffyTargetHorizontal() { return diffyHPosition; }

    public double getDiffyTrueVertical() { return currVDiffy; }

    public double getDiffyTrueHorizontal() { return currHDiffy; }

    public double getDiffyLeftEncoder() {
        return diffyLeftServo.getAbsolutePosition();
    }

    public double getDiffyRightEncoder() {
        return diffyRightServo.getAbsolutePosition();
    }

    public Outtake_CR(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;

        diffyLeftServo = new AxonPlusServo(hardwareMap.get(CRServo.class, "diffyLeft"),
                hardwareMap.get(AnalogInput.class, "diffyLeftEncoder"));
        diffyRightServo = new AxonPlusServo(hardwareMap.get(CRServo.class, "diffyRight"),
                hardwareMap.get(AnalogInput.class, "diffyRightEncoder"));
        fourBarServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeFourBar"));
        rotateServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeRotate"));
        clawServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeClaw"));
        sensorUp = new OPDistanceSensor(hardwareMap.get(DistanceSensor.class, "sensorUp"));
        sensorDown = new OPDistanceSensor(hardwareMap.get(DistanceSensor.class, "sensorDown"));

        currVDiffy = lastVDiffy = diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
        currHDiffy = lastHDiffy = diffyHPosition = HDIFFY_CENTER_POS;

        outtakeState = OuttakeState.TRANSFER;
        lastOuttakeState = OuttakeState.TRANSFER;
        clawState = ClawState.OPEN;
        rotateState = RotateState.CENTER;
        diffyHState = DiffyHorizontalState.CENTER;

        diffyLeftServo.setAbsolutePosition((VDIFFY_TRANSFER_POS + HDIFFY_CENTER_POS) * 360);
        diffyRightServo.setAbsolutePosition((VDIFFY_TRANSFER_POS - HDIFFY_CENTER_POS) * 360);
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
            case TRANSFER:
                fourBarServo.setPosition(FOURBAR_TRANSFER_POS);
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_POS;
                break;
            case TRANSFER_PREP:
                if (currVDiffy == diffyVPosition) {
                    fourBarServo.setPosition(FOURBAR_TRANSFER_PREP_POS);
                }
                diffyHPosition = HDIFFY_CENTER_POS;
                diffyVPosition = VDIFFY_TRANSFER_PREP_POS;
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
                        yawControlFourbarPos = FOURBAR_SCORE_POS;
                        diffyHPosition = HDIFFY_CENTER_POS;
                        break;
                    case RIGHT:
                        yawControlFourbarPos = FOURBAR_SCORE_ANGLED_POS;
                        diffyHPosition = HDIFFY_RIGHT_POS;
                        break;
                }

                if (currVDiffy == diffyVPosition &&
                        currHDiffy == diffyHPosition) {
                    if (sensorUp.getDistance() < sensorDown.getDistance()) {
                        yawControlFourbarPos -= yawControlGain;
                    } else if (sensorDown.getDistance() < sensorUp.getDistance()) {
                        yawControlFourbarPos += yawControlGain;
                    }
                }
                fourBarServo.setPosition(yawControlFourbarPos);
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
