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
import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
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
    private ElapsedTime blockedIntake = new ElapsedTime(0);
    private ElapsedTime transferTimer = new ElapsedTime(0);
    private ElapsedTime scoreTimer = new ElapsedTime(0);

    // Telemetry Switches
    public static boolean intakeTelemetry = true;
    public static boolean outtakeTelemetry = true;
    public static boolean elevatorTelemetry = true;

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
        // here
        robot.climb.climbState = Climb.ClimbState.PASSIVE;
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
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1, false, false);
                    break;
                case SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7, false, false);
                    break;
            }

        }

        // INTAKE
        if (robot.intake.intakeMode == Intake.IntakeMode.IN && Intake.blockedThreshold <= robot.intake.getVelocity()) {
            robot.intake.intakeMode = Intake.IntakeMode.OUT;
            blockedIntake.reset();
        }
        if (0.35 <= blockedIntake.seconds() && blockedIntake.seconds() <= 0.5 && robot.intake.intakeMode == Intake.IntakeMode.OUT) {
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }

        if (stickyGamepad1.a) {
            robot.intake.dropdownState = Intake.DropdownState.UP;
        }
        if (stickyGamepad1.b) {
            robot.intake.dropdownState = Intake.DropdownState.DOWN;
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.dropdownState = Intake.DropdownState.MANUAL;
            if (robot.intake.lastDropdownState == Intake.DropdownState.UP) {
                robot.intake.manualPosition = Intake.INTAKE_DROPDOWN_UP;
            }
            if (robot.intake.lastDropdownState == Intake.DropdownState.DOWN) {
                robot.intake.manualPosition = Intake.INTAKE_DROPDOWN_DOWN;
            }
            robot.intake.manualPosition += gamepad1.right_trigger * 0.02;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intake.dropdownState = Intake.DropdownState.MANUAL;
            if (robot.intake.lastDropdownState == Intake.DropdownState.UP) {
                robot.intake.manualPosition = Intake.INTAKE_DROPDOWN_UP;
            }
            if (robot.intake.lastDropdownState == Intake.DropdownState.DOWN) {
                robot.intake.manualPosition = Intake.INTAKE_DROPDOWN_DOWN;
            }
            robot.intake.manualPosition += -gamepad1.left_trigger * 0.02;
        }

        if (0.5 < blockedIntake.seconds()) {
            switch (robot.intake.intakeMode) {
                case IN:
                    if (stickyGamepad1.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    }
                    if (stickyGamepad1.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    }
                    break;
                case IDLE:
                    if (stickyGamepad1.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    }
                    if (stickyGamepad1.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    }
                    break;
                case OUT:
                    if (stickyGamepad1.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    }
                    if (stickyGamepad1.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    }
                    break;
            }
        }

        // ELEVATOR
        if (gamepad2.right_trigger > 0.1) {
            robot.elevator.elevatorState = Elevator.ElevatorState.MANUAL;
            robot.elevator.manualPower = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            robot.elevator.elevatorState = Elevator.ElevatorState.MANUAL;
            robot.elevator.manualPower = -gamepad2.left_trigger;
        } else if (robot.elevator.elevatorState == Elevator.ElevatorState.MANUAL) {
            robot.elevator.manualOffset = robot.elevator.getCurrentPosition() - robot.elevator.getTargetPosition();
            robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
            robot.elevator.manualPower = Elevator.IDLE_POWER;
        }

        // OUTTAKE

        // ROTATE STATE
        if (stickyGamepad2.dpad_left) {
            robot.outtake.rotateState = Outtake.RotateState.LEFT;
        }
        if (stickyGamepad2.dpad_up) {
            robot.outtake.rotateState = Outtake.RotateState.CENTER;
        }
        if (stickyGamepad2.dpad_right) {
            robot.outtake.rotateState = Outtake.RotateState.RIGHT;
        }

        // H DIFFY STATE
        if (stickyGamepad2.x) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.LEFT;
        }
        if (stickyGamepad2.y) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.CENTER;
        }
        if (stickyGamepad2.b) {
            robot.outtake.diffyHState = Outtake.DiffyHortizontalState.RIGHT;
        }

        switch (robot.outtake.outtakeState) {
            case TRANSFER:
                if (stickyGamepad2.left_bumper) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    transferTimer.reset();
                }
                if (0.2 < transferTimer.seconds() && transferTimer.seconds() < 0.3) {
                    robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
                    robot.elevator.setTargetHeight(Elevator.TargetHeight.SCORE);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                }
                break;
            case SCORE:
                if (stickyGamepad2.left_bumper) {
                    robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
                    robot.elevator.setTargetHeight(Elevator.TargetHeight.TRANSFER);
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                }

                // SCORING OPTIONS
                if (stickyGamepad2.right_bumper || stickyGamepad2.a) {
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                    scoreTimer.reset();
                }
                break;
        }
        if (0.2 < scoreTimer.seconds() && scoreTimer.seconds() < 0.3) {
            robot.elevator.elevatorState = Elevator.ElevatorState.AUTOMATIC;
            robot.elevator.setTargetHeight(Elevator.TargetHeight.SCORE);
            robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
        }

        // manual
        if (gamepad2.right_stick_y != 0) {
            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualVDiffy += gamepad2.right_stick_y;
        }
        if (gamepad2.right_stick_x != 0) {
            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualHDiffy += gamepad2.right_stick_x;
        }
        if (gamepad2.right_stick_y != 0) {
            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualFourbarPos += gamepad2.right_stick_y;
        }
//        if (gamepad2.right_stick_x != 0) {
//            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
//            robot.outtake.getManualValues();
//            robot.outtake.manualFourbarPos += gamepad2.right_stick_x;
//        }

        // Climber State Machine
        switch (robot.climb.climbState) {
            case PASSIVE:
                if (stickyGamepad1.dpad_left) {
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
        if (intakeTelemetry) {
            telemetry.addData("intake_motor_velocity_deg", robot.intake.getVelocity());
            telemetry.addData("intake_mode", robot.intake.intakeMode);
            telemetry.addData("intake_motor_amps", robot.intake.getCurrent());
        }

        if (outtakeTelemetry) {
            telemetry.addData("outtake_state", robot.outtake.outtakeState);
            telemetry.addData("diffy_horizontal", robot.outtake.getDiffyHorizontal());
            telemetry.addData("diffy_vertical", robot.outtake.getDiffyVertical());
        }

        if (elevatorTelemetry) {
            telemetry.addData("elevator_state", robot.elevator.elevatorState);
            telemetry.addData("target_height", robot.elevator.targetHeight);
            telemetry.addData("elevator_encoder", robot.elevator.getCurrentPosition());
            telemetry.addData("elevator_left_motor_amps", robot.elevator.getCurrentLeft());
            telemetry.addData("elevator_right_motor_amps", robot.elevator.getCurrentRight());
        }

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