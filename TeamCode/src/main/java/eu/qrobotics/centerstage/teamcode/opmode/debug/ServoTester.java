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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public  double maxVol=0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        axon = new AxonPlusServo(hardwareMap.get(CRServo.class, "intakeServo"),
                                hardwareMap.get(AnalogInput.class, "intakeEncoder"));

        waitForStart();

        axon.reverseServo();

        axon.setPower(0.1);
        sleep(100);

        while (!isStopRequested()) {
            axon.update();

            pidfController.setTargetPosition(targetPos);

            if(gamepad1.dpad_up) {
                axon.setPower(0.2);
            }
            else if(gamepad1.dpad_down){
                axon.setPower(-0.2);
            }
            else {
                axon.setPower(0);
            }

            //axon.setPower(pidfController.update(axon.getAbsolutePosition()));
            maxVol=Math.max(maxVol,axon.delegateEncoder.getVoltage());

            multipleTelemetry.addData("Encoder position", axon.getRelativePosition());
            multipleTelemetry.addData("Absolute position", axon.getAbsolutePosition());
            multipleTelemetry.addData("Target position", targetPos);
            multipleTelemetry.addData("Voltage ", axon.getVoltage());
            multipleTelemetry.addData("Rotations ", axon.rotations);
            multipleTelemetry.addData("average ", axon.averageAngle);
            multipleTelemetry.update();
        }
    }
}
