package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@Config
@TeleOp(name = "ServoTester")
public class ServoTester extends LinearOpMode {
    private AxonPlusServo axon;


    public static PIDCoefficients pidCoefficients = new PIDCoefficients(2, 0, 0);
    private PIDFController pidfController = new PIDFController(pidCoefficients);

    public static double targetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        axon = new AxonPlusServo(hardwareMap.get(CRServo.class, "intakeServo"),
                                hardwareMap.get(AnalogInput.class, "intakeEncoder"));

        waitForStart();

        while (!isStopRequested()) {
            axon.update();

            pidfController.setTargetPosition(targetPos);
            axon.setPower(pidfController.update(axon.getAbsolutePosition()));

            telemetry.addData("Encoder position", axon.getRelativePosition());
            telemetry.addData("Absolute position", axon.getAbsolutePosition());
            telemetry.addData("Target position", targetPos);
            telemetry.update();
        }
    }
}
