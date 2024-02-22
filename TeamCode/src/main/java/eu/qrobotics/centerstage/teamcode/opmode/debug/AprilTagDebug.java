///* Copyright (c) 2023 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package eu.qrobotics.centerstage.teamcode.opmode.debug;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.concurrent.TimeUnit;
//
//import eu.qrobotics.centerstage.teamcode.cv.ATagDetector;
//import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
//import eu.qrobotics.centerstage.teamcode.opmode.auto.regionals.red.trajectories.TrajectoryRedCloseCS;
//import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
//import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
//import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
//import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;
//
//@Config
//@TeleOp(name="AprilTagDebug")
//public class AprilTagDebug extends LinearOpMode {
//    // Adjust these numbers to suit your robot.
//    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
//
//    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
//    //  applied to the drive motors to correct the error.
//    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
//    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//    private CachingDcMotorEx leftFrontDrive   = null;  //  Used to control the left front drive wheel
//    private CachingDcMotorEx rightFrontDrive  = null;  //  Used to control the right front drive wheel
//    private CachingDcMotorEx leftBackDrive    = null;  //  Used to control the left back drive wheel
//    private CachingDcMotorEx rightBackDrive   = null;  //  Used to control the right back drive wheel
//
//    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
//    public static int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
//
//    Robot robot;
//
//    ATagDetector aTagDetector;
//    StickyGamepad stickyGamepad1 = null;
//
//    @Override public void runOpMode()
//    {
//        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//
//        // Initialize the Apriltag Detection process
//        robot = new Robot(this, false);
//        robot.drive.setPoseEstimate(TrajectoryRedCloseCS.START_POSE);
//        robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
//
//        int[] portals= VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//        aTagDetector = new ATagDetector(robot, hardwareMap,portals[0]);
//        setManualExposure(6,250);
//        robot.drive.setATagDetector(aTagDetector, true);
//
//        stickyGamepad1 = new StickyGamepad(gamepad1);
//
//        // Wait for driver to press start
//
//        robot.start();
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            stickyGamepad1.update();
//            robot.drive.setMotorPowersFromGamepad(gamepad1, 1, false);
//
//           /* robot.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );*/
//
//            //aTagDetector.detect();
//
//            telemetry.addData("Detected", aTagDetector.detected);
//            //telemetry.addData("Tries: ", aTagDetector.tries);
//            if (aTagDetector.detected) {
//                Pose2d newPose = aTagDetector.estimatedPose;
//
//                telemetry.addData("X", newPose.getX());
//                telemetry.addData("Y", newPose.getY());
//                telemetry.addData("Heading", Math.toDegrees(newPose.getHeading()));
//                telemetry.addData("no of atags", aTagDetector.detectionSize);
//                telemetry.addData("time", aTagDetector.getTime());
//                telemetry.addData("Max update time", aTagDetector.maxUpdateTime);
//            }  // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//
//            if (stickyGamepad1.a) {
//                robot.drive.useAprilTagDetector = false;
//            }
//            if (stickyGamepad1.b) {
//                robot.drive.useAprilTagDetector = true;
//            }
//
//            telemetry.addData("useatag", robot.drive.useAprilTagDetector);
//
//            Pose2d poseEstimate = robot.drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
//            telemetry.update();
//        }
//        aTagDetector.close();
//        robot.stop();
//    }
//
//    /**
//     * Move robot according to desired axes motions
//     * <p>
//     * Positive X is forward
//     * <p>
//     * Positive Y is strafe left
//     * <p>
//     * Positive Yaw is counter-clockwise
//     */
//    public void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        robot.drive.setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
//    }
//
//    /**
//     * Initialize the AprilTag processor.
//     */
//
//
//    /*
//     Manually set the camera gain and exposure.
//     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
//    */
//    private void setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (aTagDetector.visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (aTagDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Webcam 2", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (aTagDetector.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                telemetry.addData("Webcam 2", "Waiting");
//                telemetry.addData("State:", aTagDetector.visionPortal.getCameraState());
//                telemetry.update();
//                sleep(50);
//            }
//            telemetry.addData("Webcam 2", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = aTagDetector.visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = aTagDetector.visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//
//    }
//
//}