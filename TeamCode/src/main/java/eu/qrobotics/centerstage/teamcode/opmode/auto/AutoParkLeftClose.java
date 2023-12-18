package eu.qrobotics.centerstage.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import eu.qrobotics.centerstage.teamcode.cv.TeamPropPipeline;
import eu.qrobotics.centerstage.teamcode.opmode.auto.trajectories.TrajectoryTest;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "#00 AParkLeftClose")
public class AutoParkLeftClose extends LinearOpMode {

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);

        robot.start();

        // fata sa intake
        robot.drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
        robot.intake.intakeMode = Intake.IntakeMode.IN;
        robot.sleep(0.2);

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.sleep(0.4);

        // dreapta
        robot.drive.setMotorPowers(0.6, -0.6, 0.6, -0.6);
        robot.sleep(0.7);

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.sleep(0.5);
        robot.intake.intakeMode = Intake.IntakeMode.OUT;
        robot.sleep(0.6);


        // spate
        robot.drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
        robot.sleep(0.3);
        robot.drive.setMotorPowers(0, 0, 0, 0);

        robot.sleep(0.5);

        // spin
        robot.drive.setMotorPowers(0.4, 0.4, -0.4, -0.4);
        robot.sleep(0.5);
        robot.drive.setMotorPowers(1, 1, -1, -1);
        robot.sleep(1.5);
        robot.drive.setMotorPowers(0, 0, 0, 0);

        robot.sleep(0.5);

        robot.stop();
    }

}
