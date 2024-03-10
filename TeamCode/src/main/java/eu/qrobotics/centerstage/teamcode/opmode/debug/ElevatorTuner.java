package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.centerstage.teamcode.hardware.OPColorSensor;

@Config
@TeleOp(name = "ElevatorTuner", group = "debug")
public class ElevatorTuner extends LinearOpMode {

    public static PIDCoefficients coefficients=new PIDCoefficients(0.00375, 0.00003, 0.0000);
    private static PIDFController controller=new PIDFController(coefficients);

    private CachingDcMotorEx motorLeft;
    private CachingDcMotorEx motorRight;

    public static double target=0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderRight"));
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (!isStopRequested()){

            controller.setTargetPosition(target);

            double power=controller.update(motorLeft.getCurrentPosition());

            motorLeft.setPower(power);
            motorRight.setPower(power);

            multipleTelemetry.addData("Target",target);
            multipleTelemetry.addData("Left current",motorLeft.getCurrentPosition());
            multipleTelemetry.addData("Right current",motorRight.getCurrentPosition());
            multipleTelemetry.update();


        }

    }
}
