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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import eu.qrobotics.centerstage.teamcode.hardware.AxonPlusServo;
import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;

@Config
@TeleOp(name = "ElevatorTester", group = "debug")
public class ElevatorTest extends LinearOpMode {
    private CachingDcMotorEx motorLeft;
    private CachingDcMotorEx motorRight;
    public static double ff = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderRight"));
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.left_trigger > 0.1) {
                motorLeft.setPower(-gamepad1.left_trigger);
                motorRight.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger  > 0.1) {
                motorLeft.setPower(gamepad1.right_trigger);
                motorRight.setPower(gamepad1.right_trigger);
            } else {
                motorLeft.setPower(ff);
                motorRight.setPower(ff);
            }

            multipleTelemetry.addData("motor right", motorRight.getCurrent(CurrentUnit.AMPS));
            multipleTelemetry.addData("motor left", motorLeft.getCurrent(CurrentUnit.AMPS));
            multipleTelemetry.update();
        }
    }
}
