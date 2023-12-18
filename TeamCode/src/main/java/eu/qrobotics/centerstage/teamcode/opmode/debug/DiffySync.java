package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.subsystems.Climb;
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@TeleOp
public class DiffySync extends OpMode {

    private CachingServo diffyLeft;
    private CachingServo diffyRight;

    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;
    MultipleTelemetry telemetry;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        diffyLeft = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyLeft"));
        diffyRight = new CachingServo(hardwareMap.get(ServoImplEx.class, "diffyRight"));

        diffyLeft.setDirection(Servo.Direction.REVERSE);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {

    }

    public double leftPos = 0.2;
    public double rightPos = 0.2;
    public double multiplier = 0.003;

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        double newLeftPos = leftPos,
                newRightPos = rightPos;

        if (gamepad1.left_stick_y != 0) {
            newLeftPos += gamepad1.left_stick_y * multiplier;
            newRightPos += gamepad1.left_stick_y * multiplier;
            if ((0.0 <= newLeftPos && newLeftPos <= 1.0) &&
                (0.0 <= newRightPos && newRightPos <= 1.0)) {
                leftPos = newLeftPos;
                rightPos = newRightPos;
            }
        } else if (gamepad1.left_stick_x != 0) {
            newLeftPos += gamepad1.left_stick_x * multiplier;
            newRightPos -= gamepad1.left_stick_x * multiplier;
            if ((0.0 <= newLeftPos && newLeftPos <= 1.0) &&
                    (0.0 <= newRightPos && newRightPos <= 1.0)) {
                leftPos = newLeftPos;
                rightPos = newRightPos;
            }
        }

        diffyLeft.setPosition(leftPos);
        diffyRight.setPosition(rightPos);

        telemetry.addData("leftpos", leftPos);
        telemetry.addData("rightpos", rightPos);
        telemetry.addData("vdiffy", (leftPos + rightPos) / 2.0);
        telemetry.addData("hdiffy", (leftPos - rightPos) / 2.0);

        telemetry.update();
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }
    @Override
    public void stop() {

    }
}