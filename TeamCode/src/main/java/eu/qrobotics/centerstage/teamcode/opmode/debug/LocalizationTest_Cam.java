package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;


@TeleOp(name = "LocalizationTest_Cam", group = "debug")
public class LocalizationTest_Cam extends LinearOpMode {

    private ATagDetector aprilDetector;

    private boolean useTag = false;

    private StickyGamepad stickyGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.intake.dropdownState = Intake.DropdownState.UP;

        stickyGamepad = new StickyGamepad(gamepad1);

        initCamera();

        waitForStart();

        while (!isStopRequested()) {
            robot.drive.setMotorPowersFromGamepad(gamepad1, 1, false, false);
            aprilDetector.detect();

            if (stickyGamepad.y) {
                aprilDetector.detect();
                if (aprilDetector.detected) {
                    Pose2d poseEstimate = new Pose2d(aprilDetector.estimatedPose.getX(),
                            aprilDetector.estimatedPose.getY(), Math.toRadians(aprilDetector.estimatedPose.getHeading()));
                    poseEstimate=robot.drive.cameraPoseToDrive(poseEstimate);
                    robot.drive.setPoseEstimate(poseEstimate);
                }
            }

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addLine("");
            telemetry.addLine("ATAG");

            poseEstimate = aprilDetector.estimatedPose;
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            robot.drive.update();
            stickyGamepad.update();
        }
    }

    public void initCamera() {
        int[] portals = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        aprilDetector = new ATagDetector(null, hardwareMap, portals[0]);
        if (aprilDetector.visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 2", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (aprilDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 2", "Waiting");
                telemetry.addData("State", aprilDetector.visionPortal.getCameraState());
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Webcam 2", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = aprilDetector.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = aprilDetector.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(120);
            sleep(20);
            sleep(20);
            WhiteBalanceControl whiteBalanceControl = aprilDetector.visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
                sleep(50);
            }
            whiteBalanceControl.setWhiteBalanceTemperature(2900);
            sleep(20);
        }
    }

}
