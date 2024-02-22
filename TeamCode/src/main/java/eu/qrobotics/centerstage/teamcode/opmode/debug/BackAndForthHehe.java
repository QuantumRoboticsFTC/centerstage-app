package eu.qrobotics.centerstage.teamcode.opmode.debug;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class BackAndForthHehe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
        robot.intake.dropdownState = Intake.DropdownState.UP;
        boolean fataSpate = false;
        boolean normal = false;
        String msgString;

        waitForStart();

        while (!isStopRequested()) {
            if (normal) {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                msgString = "Normal";
            } else if (fataSpate) {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                0,
                                0
                        )
                );
                msgString = "Fata Spate";
            } else {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                0,
                                0
                        )
                );
                msgString = "Strafer";
            }

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("robotul merge", msgString);
            telemetry.update();
        }
    }
}
