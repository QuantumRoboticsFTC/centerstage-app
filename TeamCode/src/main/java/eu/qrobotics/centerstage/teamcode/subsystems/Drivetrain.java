package eu.qrobotics.centerstage.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;

import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import eu.qrobotics.centerstage.teamcode.hardware.OPColorSensor;
import eu.qrobotics.centerstage.teamcode.util.DashboardUtil;
import eu.qrobotics.centerstage.teamcode.util.MecanumUtil;

import static eu.qrobotics.centerstage.teamcode.subsystems.DriveConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Drivetrain extends MecanumDrive implements Subsystem {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        TURN,
        FOLLOW_TRAJECTORY,
        IDLE
    }

    public enum Localization {
        ODOMETRY,
        ATAG,
        APRILODO
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    public TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private Robot robot;
    private boolean isAutonomous;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imuChub;
    private IMU imuEhub;
    public static double initPitchChub;
    public static double initPitchEhub;
    private OPColorSensor sensorLeft;
    private OPColorSensor sensorRight;
    private double threshold = 10.0;

    private IMU.Parameters IMUParameters;

    private double[] motorPowers;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    public double rotateScale = 0.8;

    public boolean fieldCentric = false;

    public ATagDetector aTagDetector;
    public boolean useAprilTagDetector = false;
    /* angle formal definition is: from robot heading to robot-camera line, measured trigonometrically */


    public Localization localization;

    Drivetrain(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.robot = robot;
        this.isAutonomous = isAutonomous;

        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        mode = Mode.IDLE;
        localization = Localization.ODOMETRY;

        // Initialize autonomous specific stuff
        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        if(isAutonomous) {
            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                    new Pose2d(0.25, 0.25, Math.toRadians(0.0)), 0.3);
        }
        motorPowers = new double[]{0.0, 0.0, 0.0, 0.0};

        IMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imuChub = hardwareMap.get(IMU.class, "imu");
        imuChub.initialize(IMUParameters);
        initPitchChub = 0;

        imuEhub = hardwareMap.get(IMU.class, "bbno$");
        imuEhub.initialize(IMUParameters);
        initPitchEhub = 0;

        poseHistory = new LinkedList<>();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

//        sensorLeft = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensorLeft"));
//        sensorRight = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensorRight"));

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        setLocalizer(new IMUOdo(hardwareMap, this));

    }

    public void updateSensors() {
        sensorLeft.update();
        sensorRight.update();
    }

    public double getAlphaLeft() {
        return sensorLeft.alpha();
    }

    public double getAlphaRight() {
        return sensorRight.alpha();
    }

    public double getDistanceLeft() {
        return sensorLeft.getDistance();
    }

    public double getDistanceRight() {
        return sensorRight.getDistance();
    }

    public boolean lineLeft() {
        return sensorLeft.alpha() <= threshold;
    }

    public boolean lineRight() {
        return sensorRight.alpha() <= threshold;
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public void cancelTrajectory() {
        mode = Mode.IDLE;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return isAutonomous ? follower.getLastError() : new Pose2d();
        }
        throw new AssertionError();
    }

    public void setMotorPowersFromGamepad(Gamepad gg, double scale, boolean customCurve) {
        setMotorPowersFromGamepad(gg, scale, true, customCurve);
    }

    public void setMotorPowersFromGamepad(Gamepad gg, double scale, boolean reverseFront, boolean customCurve) {
        MecanumUtil.Motion motion;

        double left_stick_x = gg.left_stick_x;
        double left_stick_y = gg.left_stick_y;
        double right_stick_x = rotateScale * gg.right_stick_x;
        double right_stick_y = rotateScale * gg.right_stick_y;

        motion = MecanumUtil.joystickToMotion(left_stick_x, left_stick_y,
                right_stick_x, right_stick_y, reverseFront, customCurve);

        if (fieldCentric) {
            motion = motion.toFieldCentricMotion(getPoseEstimate().getHeading());
        }
        MecanumUtil.Wheels wh = MecanumUtil.motionToWheelsFullSpeed(motion).scaleWheelPower(scale); // Use full forward speed on 19:1 motors
        motorPowers[0] = wh.frontLeft;
        motorPowers[1] = wh.backLeft;
        motorPowers[2] = wh.backRight;
        motorPowers[3] = wh.frontRight;
    }

    public double[] getMotorPower() {
        return motorPowers;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;

        updatePoseEstimate();

        if (aTagDetector != null) {
            aTagDetector.detect();
        }

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        if (!isAutonomous) {
            setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

            if (useAprilTagDetector) {
                Pose2d newPose = new Pose2d(aTagDetector.estimatedPose.vec().minus(cameraPose.vec().rotated(aTagDetector.estimatedPose.getHeading() - cameraPose.getHeading())), aTagDetector.estimatedPose.getHeading());
                setPoseEstimate(newPose);
                setPoseEstimate(newPose);

                packet.put("ATag x", newPose.getX());
                packet.put("ATag y", newPose.getY());
                packet.put("ATag heading (deg)", Math.toDegrees(newPose.getHeading()));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawRobot(fieldOverlay, newPose);
                dashboard.sendTelemetryPacket(packet);
            } else {
                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawRobot(fieldOverlay, currentPose);
                dashboard.sendTelemetryPacket(packet);
            }
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));

        if(useAprilTagDetector) {
            if (aTagDetector != null) {
                packet.put("Debug ATag", aTagDetector.debugText);
                if (aTagDetector.detected) {
                    Pose2d newPose = new Pose2d(aTagDetector.estimatedPose.vec().minus(cameraPose.vec().rotated(aTagDetector.estimatedPose.getHeading() - cameraPose.getHeading())), aTagDetector.estimatedPose.getHeading());
                    setPoseEstimate(newPose);

                    packet.put("ATag x", newPose.getX());
                    packet.put("ATag y", newPose.getY());
                    packet.put("ATag heading (deg)", Math.toDegrees(newPose.getHeading()));
                }
            }
        } else {
            packet.put("Debug april","not using");
        }

        switch (mode) {
            case IDLE:
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

//                if (!follower.isFollowing()) {
                if (follower.elapsedTime() >= follower.getTrajectory().duration()) {
                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        packet.put("Battery", batteryVoltageSensor.getVoltage());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(),
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER
        );
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(),
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER
        );
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public Pose2d cameraPoseToDrive(Pose2d aTagPose){
        return new Pose2d(aTagPose.vec().
                minus(cameraPose.vec().rotated(aTagPose.getHeading() - cameraPose.getHeading())),
                aTagPose.getHeading());
    }

    public void setATagDetector(ATagDetector aTagDetector, boolean useATagDetector) {
        this.aTagDetector = aTagDetector;
        this.useAprilTagDetector = useATagDetector;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(motorPowers[0] = v);
        leftRear.setPower(motorPowers[1] = v1);
        rightRear.setPower(motorPowers[2] = v2);
        rightFront.setPower(motorPowers[3] = v3);
    }

    public double getPitchValueChub() {
        return imuChub.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS) - initPitchChub;
    }

    public double getPitchValueEhub() {
        return imuEhub.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS) - initPitchEhub;
    }

    public double getPitchValue() {
        return getPitchValueEhub();
    }

    @Override
    public double getRawExternalHeading() {
        return imuEhub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        return 0;
    }

    // CHUB
    public double getTruePitchChub() {
        return imuChub.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
    }

    public double getTrueYawChub() {
        return imuChub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getTrueRollChub() {
        return imuChub.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
    }

    // EHUB
    public double getTruePitchEhub() {
        return imuEhub.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
    }

    public double getTrueYawEhub() {
        return imuEhub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getTrueRollEhub() {
        return imuEhub.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imuEhub.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            robot.sleep(0.05);
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
}