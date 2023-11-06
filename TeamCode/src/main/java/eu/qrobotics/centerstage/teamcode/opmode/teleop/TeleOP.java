package eu.qrobotics.centerstage.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.centerstage.teamcode.subsystems.Climb;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@TeleOp
public class TeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW
    }

    // Previous State Region
    // Subsystems
    private Climb.ClimbState lastClimbState;


    // Timers


    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;
    MultipleTelemetry telemetry;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.log().add("Ready!");
        // TODO: Last States init
        lastClimbState = Climb.ClimbState.PASSIVE;

        // TODO: subsystem in init positions, drivetrain pose estimate
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        lastStateUpdate();

        if (stickyGamepad1.dpad_up) {
            switch (driveMode) {
                case SLOW:
                    driveMode = DriveMode.NORMAL;
                    break;
                case NORMAL:
                    driveMode = DriveMode.SLOW;
                    break;
            }
        }

        if (!robot.drive.isBusy()) {
            switch (driveMode) {
                case NORMAL:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1, false);
                    break;
                case SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7, false);
                    break;
            }

        }

        // Climber State Machine
        switch (robot.climb.climbState) {
            case PASSIVE:
                if (gamepad1.dpad_left) {
                    robot.climb.climbState = Climb.ClimbState.ACTIVE;
                    robot.climb.timer.reset();
                }
                break;
            case ACTIVE:
                if (robot.climb.getPosition() == Climb.CLIMB_ACTIVE_POSITION &&
                    robot.climb.timerRestraint < robot.climb.timer.seconds()) {
                    robot.climb.climbState = Climb.ClimbState.CLIMBED;
                }
                break;
        }

        // region Update Timers (powerplay bad alex radoo name)
        // TODO: do I do? perhaps.
        // endregion

        // region Telemetry
        // TODO: Telemetry

        addStatistics();
        telemetry.update();
        //endregion
    }

    private void lastStateUpdate() {
        lastClimbState = robot.climb.climbState;
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    private void addStatistics() {
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
    }

    @Override
    public void stop() {
        robot.stop();
    }
}