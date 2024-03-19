package eu.qrobotics.centerstage.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.centerstage.teamcode.subsystems.Endgame;
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

    Gamepad.RumbleEffect rumbleEffectTransfer;

    // Previous State Region
    // Subsystems
    private Endgame.ClimbState lastClimbState;

    public static double TOO_CLOSE_BACKDROP = 0;

    // Timers
    private ElapsedTime climbTimer = new ElapsedTime(100);
    private ElapsedTime opModeTimer = new ElapsedTime(100);
    private ElapsedTime blockedIntake = new ElapsedTime(100);
    private ElapsedTime transferDeployTimer = new ElapsedTime(100);
    private ElapsedTime transferRetractCenteredTimer = new ElapsedTime(100);
    private ElapsedTime transferRetractSideTimer = new ElapsedTime(100);
    private ElapsedTime scoreTimer = new ElapsedTime(100);
    private ElapsedTime leaveBackboardTimer = new ElapsedTime(100);
    private ElapsedTime endgameDisablePWM = new ElapsedTime(100);

    // Telemetry Switches
    public static boolean intakeTelemetry = true;
    public static boolean outtakeTelemetry = true;
    public static boolean elevatorTelemetry = true;
    public static boolean drivetrainTelemetry = true;
    public static boolean extraTelemetry = true;
    public static boolean debugTelemetry = false;

    public static double yawMultiplier = 0.01;
    public static double pwrBackdrop = 0.75;
    public static double pwrClimb = 0.85;
    public static boolean activateClimber = false;
    public static boolean firstOutsideBackstage = true;

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

        rumbleEffectTransfer = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 600)
                .build();

        telemetry.log().add("Ready!");

        // TODO: Last States init
        lastClimbState = Endgame.ClimbState.PASSIVE;

        // TODO: subsystem in init positions, drivetrain pose estimate
        // here
        robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
    }

    @Override
    public void start() {
        robot.start();
        opModeTimer.reset();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        lastStateUpdate();

        if (0.5 < opModeTimer.seconds() && opModeTimer.seconds() < 0.7) {
            robot.endgame.disableClimber();
        }

//        if (0 < robot.drive.getPoseEstimate().getX() && robot.drive.getPoseEstimate().getX() < 10) {
//            robot.intake.dropdownState = Intake.DropdownState.DOWN;
//        }
//        if (robot.drive.getPoseEstimate().getX() < 0 && robot.intake.getAbsolute() < 600 && firstOutsideBackstage) {
//            robot.intake.intakeMode = Intake.IntakeMode.IN;
//            firstOutsideBackstage = false;
//        }
//        if (robot.drive.getPoseEstimate().getX() > 0) {
//            firstOutsideBackstage = true;
//        }

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

        if (leaveBackboardTimer.seconds() < 0.175) {
            robot.drive.setMotorPowers(-pwrBackdrop, -pwrBackdrop, -pwrBackdrop, -pwrBackdrop);
        } else if (0.3 < climbTimer.seconds() && climbTimer.seconds() < 0.95) {
            robot.drive.setMotorPowers(-pwrClimb, -pwrClimb, -pwrClimb, -pwrClimb);
//        } else if (robot.outtake.getMeanSensorDistance() < TOO_CLOSE_BACKDROP &&
//            robot.outtake.outtakeState == Outtake.OuttakeState.SCORE) {
////          TODO: this
//            switch (driveMode) {
//                case NORMAL:
//                    robot.drive.setMotorPowersFromGamepad(gamepad1, 1, true, false);
//                    break;
//                case SLOW:
//                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7, true, false);
//                    break;
//            }
        } else
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

        if (stickyGamepad1.y) {
            if (robot.drive.fieldCentric) {
                robot.drive.fieldCentric = false;
            }
//            } else {
//                robot.drive.fieldCentric = true;
//            }
        }

        // INTAKE
        if (stickyGamepad1.b) {
            robot.intake.dropdownState = Intake.DropdownState.UP;
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }
        if (stickyGamepad1.a) {
            robot.intake.dropdownState = Intake.DropdownState.DOWN;
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.dropdownState = Intake.DropdownState.MANUAL;
//            if (robot.intake.lastDropdownState == Intake.DropdownState.UP) {
//                robot.intake.setPosition(Intake.INTAKE_DROPDOWN_UP);
//            }
//            if (robot.intake.lastDropdownState == Intake.DropdownState.DOWN) {
//                robot.intake.setPosition(Intake.INTAKE_DROPDOWN_DOWN);
//            }
//            robot.intake.manualPower = gamepad1.right_trigger * 0.02;
            robot.intake.manualPower = gamepad1.right_trigger * 1;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intake.dropdownState = Intake.DropdownState.MANUAL;
//            if (robot.intake.lastDropdownState == Intake.DropdownState.UP) {
//                robot.intake.setPosition(Intake.INTAKE_DROPDOWN_UP);
//            }
//            if (robot.intake.lastDropdownState == Intake.DropdownState.DOWN) {
//                robot.intake.setPosition(Intake.INTAKE_DROPDOWN_DOWN);
//            }
//            robot.intake.manualPower = -gamepad1.left_trigger * 0.02;
            robot.intake.manualPower = -gamepad1.left_trigger * 1;
        } else if (robot.intake.manualPower != 0) {
//            robot.intake.dropdownState = robot.intake.lastDropdownState;
            robot.intake.manualPower = 0;
        }

        if (stickyGamepad1.dpad_down) {
            robot.intake.resetServoPosition();
        }

        if (0.5 < blockedIntake.seconds() &&
                robot.endgame.climbState != Endgame.ClimbState.SHOOTER &&
                robot.endgame.climbState != Endgame.ClimbState.ACTIVE &&
                robot.endgame.climbState != Endgame.ClimbState.CLIMBED) {
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

//        DRIVER 1 LIFT TARGETHEIGHTS
//        if (stickyGamepad1.dpad_down) {
//            robot.elevator.targetHeight = Elevator.TargetHeight.FIRST_LINE;
//        }
//        if (stickyGamepad1.dpad_right) {
//            robot.elevator.targetHeight = Elevator.TargetHeight.SECOND_LINE;
//        }
//        if (stickyGamepad1.dpad_up) {
//            robot.elevator.targetHeight = Elevator.TargetHeight.THIRD_LINE;
//        }

        // ELEVATOR
        if (gamepad2.right_trigger > 0.1) {
            robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
            robot.elevator.manualPower = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            robot.elevator.setElevatorState(Elevator.ElevatorState.MANUAL);
            robot.elevator.manualPower = -gamepad2.left_trigger;
        } else if (robot.elevator.elevatorState == Elevator.ElevatorState.MANUAL) {
            if (robot.elevator.lastState == Elevator.ElevatorState.TRANSFER) {
                robot.elevator.groundPositionOffset += robot.elevator.getCurrentPosition() - robot.elevator.getTargetPosition(robot.elevator.lastState, robot.elevator.targetHeight);
                robot.elevator.setElevatorState(robot.elevator.lastState);
                robot.elevator.manualPower = Elevator.IDLE_POWER;
            }  else if (robot.elevator.lastState == Elevator.ElevatorState.LINES) {
                robot.elevator.manualOffset = robot.elevator.getCurrentPosition() - robot.elevator.getTargetPosition(robot.elevator.lastState, robot.elevator.targetHeight);
                robot.elevator.setElevatorState(robot.elevator.lastState);
                robot.elevator.manualPower = Elevator.IDLE_POWER;
            }
        }

        if (stickyGamepad2.dpad_down) {
            switch (robot.elevator.targetHeight) {
                case FIRST_LINE:
                    robot.elevator.targetHeight = Elevator.TargetHeight.SECOND_LINE;
                    break;
                case SECOND_LINE:
                    robot.elevator.targetHeight = Elevator.TargetHeight.THIRD_LINE;
                    break;
                case THIRD_LINE:
                    robot.elevator.targetHeight = Elevator.TargetHeight.FIRST_LINE;
                    break;
            }
        }

        // OUTTAKE

        // ROTATE STATE
        if (stickyGamepad2.dpad_left) {
            if (robot.outtake.rotateState != Outtake.RotateState.LEFT45) {
                robot.outtake.rotateState = Outtake.RotateState.LEFT45;
            } else {
                robot.outtake.rotateState = Outtake.RotateState.LEFT;
            }
        }
        if (stickyGamepad2.dpad_up) {
            robot.outtake.rotateState = Outtake.RotateState.CENTER;
        }
        if (stickyGamepad2.dpad_right) {
            if (robot.outtake.rotateState != Outtake.RotateState.RIGHT45) {
                robot.outtake.rotateState = Outtake.RotateState.RIGHT45;
            } else {
                robot.outtake.rotateState = Outtake.RotateState.RIGHT;
            }
        }

        // H DIFFY STATE
        if (stickyGamepad2.x) {
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
        }
        if (stickyGamepad2.y) {
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
        }
        if (stickyGamepad2.b) {
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
        }

        if (stickyGamepad2.right_bumper) {
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }

        switch (robot.outtake.outtakeState) {
            case TRANSFER_PREP:
                robot.outtake.fourBarState = Outtake.FourBarState.TARGET;
                if (stickyGamepad2.right_bumper) {
                    transferDeployTimer.reset();
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                }
                if (robot.intake.pixelCount() == 2) {
                    gamepad1.runRumbleEffect(rumbleEffectTransfer);
                    if (robot.intake.intakeMode == Intake.IntakeMode.IDLE) {
                        gamepad2.runRumbleEffect(rumbleEffectTransfer);
                        transferDeployTimer.reset();
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                        robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
                    }
                }
                break;
            case TRANSFER:
                robot.outtake.fourBarState = Outtake.FourBarState.TARGET;
                if (0.2 < transferDeployTimer.seconds() && transferDeployTimer.seconds() < 0.3) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                }
                if (stickyGamepad2.right_bumper) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    transferDeployTimer.reset();
                }
                if (stickyGamepad2.left_bumper) {
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                }
                break;
            case ABOVE_TRANSFER:
                robot.outtake.fourBarState = Outtake.FourBarState.TARGET;
                if (0.3 < transferDeployTimer.seconds() && transferDeployTimer.seconds() < 0.4) {
                    robot.elevator.setElevatorState(Elevator.ElevatorState.LINES);
                    robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
                }

                if (0.4 < transferRetractCenteredTimer.seconds() && transferRetractCenteredTimer.seconds() < 0.6) {
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                    robot.outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
                }
                break;
            case SCORE:
                if (stickyGamepad2.left_bumper) {
                    if (robot.outtake.diffyHState == Outtake.DiffyHorizontalState.CENTER) {
                        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                        transferRetractCenteredTimer.reset();
                    } else {
                        transferRetractSideTimer.reset();
                    }

                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                }

                if (0.5 < transferRetractSideTimer.seconds() &&
                        transferRetractSideTimer.seconds() < 0.7) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    transferRetractCenteredTimer.reset();
                }

                // SCORING OPTIONS
                if (stickyGamepad2.a) {
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                    scoreTimer.reset();
                }
                break;
            case MANUAL:
                if (stickyGamepad2.right_bumper) {
                    transferDeployTimer.reset();
                }
                if (0.1 < transferDeployTimer.seconds() && transferDeployTimer.seconds() < 0.3) {
                    robot.outtake.clawState = Outtake.ClawState.CLOSED;
                }
                if (0.4 < transferDeployTimer.seconds() && transferDeployTimer.seconds() < 0.6) {
                    transferDeployTimer.reset();
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                }

                if (stickyGamepad2.a) {
                    robot.outtake.clawState = Outtake.ClawState.OPEN;
                    scoreTimer.reset();
                }

                if (stickyGamepad2.left_bumper) {
                    if (robot.outtake.diffyHState == Outtake.DiffyHorizontalState.CENTER) {
                        robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                        transferRetractCenteredTimer.reset();
                    } else {
                        transferRetractSideTimer.reset();
                    }

                    robot.outtake.rotateState = Outtake.RotateState.CENTER;
                    robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
                    robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
                }

                if (0.5 < transferRetractSideTimer.seconds() &&
                        transferRetractSideTimer.seconds() < 0.7) {
                    robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                    transferRetractCenteredTimer.reset();
                }
                break;
        }
        if (0.2 < scoreTimer.seconds() && scoreTimer.seconds() < 0.3) {
            robot.drive.setMotorPowers(-pwrBackdrop, -pwrBackdrop, -pwrBackdrop, -pwrBackdrop);
            leaveBackboardTimer.reset();
        }
        if (0.45 < scoreTimer.seconds() && scoreTimer.seconds() < 0.55) {
            if (robot.outtake.diffyHState == Outtake.DiffyHorizontalState.CENTER) {
                robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
                transferRetractCenteredTimer.reset();
            } else {
                transferRetractSideTimer.reset();
            }

            robot.outtake.rotateState = Outtake.RotateState.CENTER;
            robot.outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
            robot.elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
        }
        if (0.5 < transferRetractSideTimer.seconds() &&
                transferRetractSideTimer.seconds() < 0.7) {
            robot.outtake.outtakeState = Outtake.OuttakeState.ABOVE_TRANSFER;
            transferRetractCenteredTimer.reset();
        }

        // manual
        if (gamepad2.left_stick_y != 0) {
            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualHDiffy += gamepad2.left_stick_y * 0.005;
        }
        if (gamepad2.left_stick_x != 0) {
            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualVDiffy += gamepad2.left_stick_x * -0.005;
        }

//        if (gamepad2.left_stick_y != 0) {
//            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
//            robot.outtake.getManualValues();
//            robot.outtake.manualFourbarPos += gamepad2.left_stick_y * 0.005;
//        }

        if (gamepad2.right_stick_x != 0) {
//            robot.outtake.outtakeState = Outtake.OuttakeState.MANUAL;
            robot.outtake.fourBarState = Outtake.FourBarState.MANUAL;
            robot.outtake.getManualValues();
            robot.outtake.manualFourbarPos += -gamepad2.right_stick_x * yawMultiplier;
        }

        // Endgame State Machine
        if (!activateClimber &&
                (stickyGamepad1.dpad_left ||
                stickyGamepad1.dpad_right)) {
            activateClimber = true;
            robot.endgame.enableClimber();
        }

        switch (robot.endgame.climbState) {
            case PASSIVE:
                if (stickyGamepad1.dpad_right) {
                    robot.endgame.climbState = Endgame.ClimbState.SHOOTER;
                }
                break;
            case SHOOTER:
                if (stickyGamepad1.dpad_right || stickyGamepad1.right_bumper) {
                    switch (robot.endgame.droneState) {
                        case PASSIVE:
                            robot.endgame.droneState = Endgame.DroneState.ACTIVE;
                            robot.drive.initPitchChub = robot.drive.getPitchValueChub();
                            robot.drive.initPitchEhub = robot.drive.getPitchValueEhub();
                            break;
                        case ACTIVE:
                            robot.endgame.climbState = Endgame.ClimbState.ACTIVE;
                            robot.intake.dropdownState = Intake.DropdownState.UP;
                            endgameDisablePWM.reset();
                            break;
                    }
                }
                if (stickyGamepad1.dpad_left || stickyGamepad1.left_bumper) {
                    robot.endgame.climbState = Endgame.ClimbState.PASSIVE;
                }
                break;
            case ACTIVE:
                if (stickyGamepad1.dpad_right || stickyGamepad1.right_bumper) {
                    climbTimer.reset();
                }
                break;
        }

        if (0.8 < climbTimer.seconds() && climbTimer.seconds() < 0.9) {
            transferDeployTimer.reset();
            robot.outtake.outtakeState = Outtake.OuttakeState.SCORE;
            robot.endgame.climbState = Endgame.ClimbState.CLIMBED;
            robot.elevator.setElevatorState(Elevator.ElevatorState.CLIMBED);
        }

        if (0.6 < endgameDisablePWM.seconds() && endgameDisablePWM.seconds() < 0.7) {
            robot.endgame.disableClimber();
        }

        // region Telemetry
        // TODO: Telemetry
        telemetry.addLine("<----> INTAKE <---->");
        if (intakeTelemetry) {
            telemetry.addData("distance 1", robot.intake.getDistance1());
            telemetry.addData("distance 2", robot.intake.getDistance2());
            telemetry.addData("distance 3", robot.intake.getDistance3());
            telemetry.addData("transfer state", robot.intake.getTransferState());
            telemetry.addData("intake_motor_velocity_deg", robot.intake.getVelocity());
            telemetry.addData("intake mode", robot.intake.intakeMode);
            telemetry.addData("dropdown mode", robot.intake.dropdownMode);
            telemetry.addData("dropdown state", robot.intake.dropdownState);
            telemetry.addData("pixel count", robot.intake.pixelCount());
            telemetry.addData("absolute value", robot.intake.getAbsolute());
            telemetry.addData("relative value", robot.intake.getRelative());
            telemetry.addData("target value", robot.intake.getTargetValue());
//            if (debugTelemetry) {
//                telemetry.addData("intake_motor_amps", robot.intake.getCurrent());
//            }
        }
        telemetry.addLine("<----> OUTTAKE <---->");
        if (outtakeTelemetry) {
            telemetry.addData("outtake_state", robot.outtake.outtakeState);
            telemetry.addData("horizontal state", robot.outtake.diffyHState);
            telemetry.addData("sensor up", robot.outtake.getSensorUp());
            telemetry.addData("sensor down", robot.outtake.getSensorDown());
//            telemetry.addData("diffy left encoder", robot.outtake.getDiffyLeftEncoder() / 360.0);
//            telemetry.addData("diffy right encoder", robot.outtake.getDiffyRightEncoder() / 360.0);
            telemetry.addData("diffy target vertical", robot.outtake.getDiffyTargetVertical());
            telemetry.addData("diffy true vertical", robot.outtake.getDiffyTrueVertical());
            telemetry.addData("diffy target horizontal", robot.outtake.getDiffyTargetHorizontal());
            telemetry.addData("diffy true horizontal", robot.outtake.getDiffyTrueHorizontal());
            telemetry.addData("diffy H state", robot.outtake.diffyHState);
            telemetry.addData("rotate state", robot.outtake.rotateState);
            telemetry.addData("rotate value", robot.outtake.getRotatePosition());
            telemetry.addData("claw state", robot.outtake.clawState);
//            if (debugTelemetry) {
//                telemetry.addData("manual rotate position", robot.outtake.manualRotatePos);
//                telemetry.addData("manual fourbar pos", robot.outtake.manualFourbarPos);
//                telemetry.addData("diffy_last_horizontal", robot.outtake.lastHDiffy);
//                telemetry.addData("diffy_last_vertical", robot.outtake.lastVDiffy);
//                telemetry.addData("diffy_horizontal_manual", robot.outtake.manualHDiffy);
//                telemetry.addData("diffy_vertical_manual", robot.outtake.manualVDiffy);
//                telemetry.addData("diffy left", robot.outtake.manualVDiffy + robot.outtake.manualHDiffy);
//                telemetry.addData("diffy right", robot.outtake.manualVDiffy - robot.outtake.manualHDiffy);
//            }
        }
        telemetry.addLine("<----> ELEVATOR <---->");
        if (elevatorTelemetry) {
            telemetry.addData("elevator_state", robot.elevator.elevatorState);
            telemetry.addData("liftTargetHeightState", robot.elevator.targetHeight);
            telemetry.addData("liftTargetHeightPos", robot.elevator.getTargetPosition());
            telemetry.addData("elevator_encoder", robot.elevator.getCurrentPosition());
            telemetry.addData("power left", robot.elevator.getPowerLeft());
            telemetry.addData("power right", robot.elevator.getPowerRight());
            telemetry.addData("manual power", robot.elevator.manualPower);
            telemetry.addData("manual offset", robot.elevator.manualOffset);
            telemetry.addData("ground offset", robot.elevator.groundPositionOffset);
            telemetry.addData("left trigger", gamepad2.left_trigger);
            telemetry.addData("right trigger", gamepad2.right_trigger);
//            if (debugTelemetry) {
//                telemetry.addData("elevator_left_motor_amps", robot.elevator.getCurrentLeft());
//                telemetry.addData("elevator_right_motor_amps", robot.elevator.getCurrentRight());
//            }
        }
        telemetry.addLine("<----> DRIVETRAIN <---->");
        if (drivetrainTelemetry) {
            telemetry.addData("field centric", robot.drive.fieldCentric);
            telemetry.addData("alpha left", robot.drive.getAlphaLeft());
            telemetry.addData("alpha right", robot.drive.getAlphaRight());
            telemetry.addData("distance left", robot.drive.getDistanceLeft());
            telemetry.addData("distance right", robot.drive.getDistanceRight());
        }
        telemetry.addLine("<----> MISC <---->");
        if (extraTelemetry) {
            telemetry.addData("climb_state", robot.endgame.climbState);
            telemetry.addData("pitch chub", robot.drive.getTruePitchChub());
            telemetry.addData("pitch ehub", robot.drive.getTruePitchEhub());
            telemetry.addData("roll chub", robot.drive.getTrueRollChub());
            telemetry.addData("roll ehub", robot.drive.getTrueRollEhub());
            telemetry.addData("yaw chub", robot.drive.getTrueYawChub());
            telemetry.addData("yaw ehub", robot.drive.getTrueYawEhub());
        }

        addStatistics();

        telemetry.update();
        //endregion
    }

    private void lastStateUpdate() {
        lastClimbState = robot.endgame.climbState;
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