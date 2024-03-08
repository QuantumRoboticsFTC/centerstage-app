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
import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@Config
@TeleOp(name = "ClimberTester")
public class ClimberDebug extends LinearOpMode {

    Endgame endgame;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        endgame = new Endgame(hardwareMap, null);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);

        waitForStart();

//        axon.reverseServo();

        sleep(100);

        while (!isStopRequested()) {

            if (stickyGamepad1.x) {
                endgame.climbState = Endgame.ClimbState.PASSIVE;
            }
            if (stickyGamepad1.a) {
                endgame.climbState = Endgame.ClimbState.SHOOTER;
            }
            if (stickyGamepad1.y) {
                endgame.climbState = Endgame.ClimbState.ACTIVE;
            }

            endgame.update();
            telemetry.update();
            stickyGamepad1.update();
        }
    }
}
