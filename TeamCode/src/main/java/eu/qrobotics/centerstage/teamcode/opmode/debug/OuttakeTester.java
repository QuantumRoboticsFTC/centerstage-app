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
@TeleOp(name = "OuttakeTester", group = "debug")
public class OuttakeTester extends LinearOpMode {
    private AxonPlusServo axon1;
    private AxonPlusServo axon2;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.01, 0, 0.000025);
    private PIDFController pidfController1 = new PIDFController(pidCoefficients);
    private PIDFController pidfController2 = new PIDFController(pidCoefficients);
    public static double ff = 0.05;
    public static double targetPosition = 0;
    public static double feedforwardSpot = 0.9;

    public static double speed = 0.4;
    public static double gain = 0.4;

    public static boolean auto = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        axon1 = new AxonPlusServo(hardwareMap.get(CRServo.class, "diffyLeft"),
                hardwareMap.get(AnalogInput.class, "diffyLeftEncoder"));
        axon2 = new AxonPlusServo(hardwareMap.get(CRServo.class, "diffyRight"),
                hardwareMap.get(AnalogInput.class, "diffyRightEncoder"));
        axon2.setDirection(-1);

        boolean reset = true;

        waitForStart();

        while (!isStopRequested()) {
            axon1.update();
            axon2.update();

            if (reset) {
                reset = false;
                axon1.setAbsolutePosition(0);
                axon2.setAbsolutePosition(0);
            }

            if (auto) {
                if (axon1.getAbsolutePosition() < feedforwardSpot) {
                    pidfController1.setTargetPosition(targetPosition);
                    axon1.setPower(ff + pidfController1.update(axon1.getAbsolutePosition()));
                    pidfController2.setTargetPosition(targetPosition);
                    axon2.setPower(ff - pidfController1.update(axon2.getAbsolutePosition()));
                } else {
                    pidfController1.setTargetPosition(targetPosition);
                    axon1.setPower(-ff + pidfController2.update(axon1.getAbsolutePosition()));
                    pidfController2.setTargetPosition(targetPosition);
                    axon2.setPower(-ff - pidfController2.update(axon2.getAbsolutePosition()));
                }
            } else {
                if (gamepad1.dpad_up) {
                    axon1.setPower(ff + speed);
                    axon2.setPower(ff + speed);
                } else if (gamepad1.dpad_down) {
                    axon1.setPower(ff - speed);
                    axon2.setPower(ff - speed);
                } else if (gamepad1.right_trigger > 0) {
                    axon1.setPower(ff + gamepad1.right_trigger * gain);
                    axon2.setPower(ff + gamepad1.right_trigger * gain);
                } else if (gamepad1.left_trigger > 0) {
                    axon1.setPower(ff - gamepad1.left_trigger * gain);
                    axon2.setPower(ff - gamepad1.left_trigger * gain);
                } else {
                    axon1.setPower(ff);
                    axon2.setPower(ff);
                }
            }

            //axon.setPower(pidfController.update(axon.getAbsolutePosition()));
            multipleTelemetry.addData("Encoder position 1", axon1.getRelativePosition());
            multipleTelemetry.addData("Absolute position 1", axon1.getAbsolutePosition());
            multipleTelemetry.addData("Encoder position 2", axon2.getRelativePosition());
            multipleTelemetry.addData("Absolute position 2", axon2.getAbsolutePosition());
            multipleTelemetry.addData("Voltage 1", axon1.getVoltage());
            multipleTelemetry.addData("Voltage 2", axon2.getVoltage());
            multipleTelemetry.update();
        }
    }
}
