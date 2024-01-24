package eu.qrobotics.centerstage.teamcode.opmode.auto.never;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#00 ATimeCloseBlue", group = "Time")
public class AutoTimeLeftCloseBlue extends LinearOpMode {
    Robot robot;
    int noDetectionFlag = -1;
    int robotStopFlag = -10; // if robot.stop while camera

    int cameraTeamProp() {
        int readFromCamera = noDetectionFlag;

        OpenCvCamera camera;
        TeamPropPipelineBlue teamPropPieline = new TeamPropPipelineBlue();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.setPipeline(teamPropPieline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // :salute:
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            readFromCamera = teamPropPieline.getTeamProp();
            telemetry.addData("getTeamProp() ", teamPropPieline.getTeamProp());
            telemetry.addData("getAvg() ", teamPropPieline.getMax());
            telemetry.addData("getCnt() ", teamPropPieline.getCount());
            telemetry.addData("isRed ", teamPropPieline.isPropRed());
            telemetry.addData("readFromCamera ", readFromCamera);
            telemetry.update();
        }

        if (isStopRequested()) {
            return robotStopFlag;
        }

        camera.closeCameraDeviceAsync(() -> {});
        return readFromCamera;
    }

    void spin(double val) {
        robot.drive.setMotorPowers(val, val, -val, -val);
    }

    void translational(double val) {
        robot.drive.setMotorPowers(-val, val, -val, val);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        int camera = cameraTeamProp();
//        int camera = 2;
//        while (!isStarted() && !isStopRequested());

        robot.start();
        robot.sleep(1);

        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
        // purple pixel
        robot.drive.setMotorPowers(-0.5, -0.5, -0.5, -0.5);
        robot.sleep(0.25);

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(0.1);

        if (camera == 2) {
            // center
            robot.drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
            robot.sleep(0.73);

            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(0.6);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            // spate un pic
            robot.drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.sleep(0.3);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);

            // spin
            spin(0.4);
            robot.sleep(0.7);
            robot.drive.setMotorPowers(0, 0, 0, 0);

            robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
            robot.sleep(0.25);
            robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            robot.sleep(0.4);

            // go to backboard
            robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            robot.sleep(1.23);
//            gotobackboard();

            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(1.2);
        } else if (camera == 3) {
            // stanga
            robot.drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
            robot.sleep(0.48);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.25);

            spin(0.4);
            robot.sleep(0.52);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.3);

            robot.drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
            robot.sleep(0.23);

            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(0.6);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            // spate un pic
            robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            robot.sleep(0.45);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);
            robot.drive.setMotorPowers(0, 0, 0, 0);

            // spin
            spin(0.4);
            robot.sleep(0.64);
            robot.drive.setMotorPowers(0, 0, 0, 0);

            // go to backboard
            robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
            robot.sleep(0.65);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);

            spin(-0.4);
            robot.sleep(0.418);
            robot.drive.setMotorPowers(0, 0, 0, 0);

            robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
            robot.sleep(0.25);
            robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            robot.sleep(0.7);

            robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            robot.sleep(0.64);
//            gotobackboard();
            robot.drive.setMotorPowers(0, 0, 0, 0);
        } else if (camera == 1) {
            // stanga
            spin(-0.4);
            robot.sleep(0.22);

            robot.drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
            robot.sleep(0.5);

            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.5);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(0.6);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            // spate un pic
            robot.drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.sleep(0.3);
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.4);

            // spin
            spin(0.4);
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
            robot.sleep(0.9);
            robot.drive.setMotorPowers(0, 0, 0, 0);

            robot.outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
            robot.sleep(0.25);
            robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            robot.sleep(0.5);

            // go to backboard
            robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            robot.sleep(1.23);
//            gotobackboard();
            robot.drive.setMotorPowers(0, 0, 0, 0);
            robot.sleep(0.4);
        }

        robot.drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
        robot.sleep(0.3);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(0.1);
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(0.2);
        robot.drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
        robot.sleep(0.04);
        robot.outtake.rotateState = Outtake.RotateState.CENTER;
        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
        robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        robot.sleep(1);

        translational(0.6);
        if (camera == 2) {
            robot.sleep(0.82);
        } else if (camera == 1) {
            robot.sleep(1.07);
        } else if (camera == 3) {
            robot.sleep(0.65);
        }
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(0.5);

        robot.drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
        robot.sleep(0.75);

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(0.5);

        robot.stop();
    }
}
