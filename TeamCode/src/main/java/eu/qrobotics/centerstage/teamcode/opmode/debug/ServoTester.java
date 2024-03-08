package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;

@Config
@TeleOp(name = "ServoTester")
public class ServoTester extends LinearOpMode {
    private AxonPlusServo axon;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.01, 0, 0.000025);
    private PIDFController pidfController = new PIDFController(pidCoefficients);
    public static double ff = 0.05;

    public static double UP_POSITION = 800;
    public static double DOWN_POSITION = 0;

    public static double INITIAL_TARGET = 180;

    public static double targetPos = UP_POSITION;
    public static double epsilon = 2;

    public static double servoToDDown = 90.0 / 800.0; // servo angle 2 dropdown angle

    // manual - not important
    public static double speed = 0.4;
    public static double gain = 0.4;
    public static double maxing = 3.25;

    public static boolean opMode = false; // false - manual; true - auto
    public static boolean wentToNeutral = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        axon = new AxonPlusServo(hardwareMap.get(CRServo.class, "intakeServo"),
                                hardwareMap.get(AnalogInput.class, "intakeEncoder"));

        waitForStart();

        axon.setAbsolutePosition(axon.getRelativePosition());

        while (!isStopRequested()) {
            axon.update();

            if (opMode && !wentToNeutral) {
                // go to 100
                pidfController.setTargetPosition(INITIAL_TARGET);
                axon.setPower(ff + pidfController.update(axon.getAbsolutePosition()));
                multipleTelemetry.addData("pidu zice", pidfController.update(axon.getAbsolutePosition()));

                if (INITIAL_TARGET - epsilon < axon.getAbsolutePosition() &&
                    axon.getAbsolutePosition() < INITIAL_TARGET + epsilon) {
                    wentToNeutral = true;
                    opMode = false;
                    axon.setAbsolutePosition(UP_POSITION);
                }
            }

            if (opMode && wentToNeutral) {
//                pidfController.setTargetPosition(targetPos);
                pidfController.setTargetPosition(DOWN_POSITION + targetPos * 1 / servoToDDown);
                axon.setPower(ff + pidfController.update(axon.getAbsolutePosition()));
            } else {
                if (gamepad1.dpad_up) {
                    axon.setPower(speed);
                } else if (gamepad1.dpad_down) {
                    axon.setPower(-speed);
                } else if (gamepad1.right_trigger > 0) {
                    axon.setPower(gamepad1.right_trigger * gain);
                } else if (gamepad1.left_trigger > 0) {
                    axon.setPower(-gamepad1.left_trigger * gain);
                } else {
                    axon.setPower(0);
                }
            }

            //axon.setPower(pidfController.update(axon.getAbsolutePosition()));
            multipleTelemetry.addData("Encoder position", axon.getRelativePosition());
            multipleTelemetry.addData("Absolute position", axon.getAbsolutePosition());
            multipleTelemetry.addData("true traget", DOWN_POSITION + targetPos * 1 / servoToDDown);
            multipleTelemetry.addData("dropdowmn angle", axon.getAbsolutePosition() * servoToDDown);
            multipleTelemetry.addData("cached position", axon.getCachedPosition());
            multipleTelemetry.addData("Target position", targetPos);
            multipleTelemetry.addData("Voltage ", axon.getVoltage());
            multipleTelemetry.addData("Voltage filtered ", Math.max(axon.getVoltage(), maxing));
            multipleTelemetry.addData("Rotations ", axon.rotations);
            multipleTelemetry.update();
        }
    }
}
