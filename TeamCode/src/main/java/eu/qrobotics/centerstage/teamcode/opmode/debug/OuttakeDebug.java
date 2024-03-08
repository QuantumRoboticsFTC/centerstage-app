//package eu.qrobotics.centerstage.teamcode.opmode.debug;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.MovingStatistics;
//
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//
//import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
//import eu.qrobotics.centerstage.teamcode.subsystems.Elevator;
//import eu.qrobotics.centerstage.teamcode.subsystems.Outtake;
//import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;
//
//@TeleOp
//public class OuttakeDebug extends OpMode {
//
//    Outtake outtake;
//
//    StickyGamepad stickyGamepad1 = null;
//    StickyGamepad stickyGamepad2 = null;
//    MultipleTelemetry telemetry;
//
//    ElapsedTime transferTimer = new ElapsedTime(0);
//    ElapsedTime leaveBackboardTimer = new ElapsedTime(0);
//    ElapsedTime scoreTimer = new ElapsedTime(0);
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
//        outtake = new Outtake(hardwareMap, null, false);
//
//        stickyGamepad1 = new StickyGamepad(gamepad1);
//        stickyGamepad2 = new StickyGamepad(gamepad2);
//
//        telemetry.log().add("Ready!");
//    }
//
//    @Override
//    public void start() {
//
//    }
//
//    @Override
//    public void loop() {
//        stickyGamepad1.update();
//        stickyGamepad2.update();
//
//
//        // OUTTAKE
//
//        // ROTATE STATE
//        if (stickyGamepad2.dpad_left) {
//            if (outtake.rotateState != Outtake.RotateState.LEFT45) {
//                outtake.rotateState = Outtake.RotateState.LEFT45;
//            } else {
//                outtake.rotateState = Outtake.RotateState.LEFT;
//            }
//        }
//        if (stickyGamepad2.dpad_up) {
//            outtake.rotateState = Outtake.RotateState.CENTER;
//        }
//        if (stickyGamepad2.dpad_right) {
//            if (outtake.rotateState != Outtake.RotateState.RIGHT45) {
//                outtake.rotateState = Outtake.RotateState.RIGHT45;
//            } else {
//                outtake.rotateState = Outtake.RotateState.RIGHT;
//            }
//        }
//
//        // H DIFFY STATE
//        if (stickyGamepad2.x) {
//            outtake.diffyHState = Outtake.DiffyHorizontalState.LEFT;
//        }
//        if (stickyGamepad2.y) {
//            outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
//        }
//        if (stickyGamepad2.b) {
//            outtake.diffyHState = Outtake.DiffyHorizontalState.RIGHT;
//        }
//
//        switch (outtake.outtakeState) {
//            case :
//                if (stickyGamepad2.right_bumper) {
//                    transferTimer.reset();
//                    outtake.outtakeState = Outtake.OuttakeState.TRANSFER;
//                }
//                break;
//            case TRANSFER:
//                if (0.45 < transferTimer.seconds() && transferTimer.seconds() < 0.65) {
//                    outtake.clawState = Outtake.ClawState.CLOSED;
//                }
//                if (0.75 < transferTimer.seconds() && transferTimer.seconds() < 0.95) {
//                    outtake.outtakeState = Outtake.OuttakeState.POST_TRANSFER;
//                }
//                if (stickyGamepad2.right_bumper) {
////                    elevator.setElevatorState(Elevator.ElevatorState.LINES);
//                    outtake.outtakeState = Outtake.OuttakeState.SCORE;
//                }
//                break;
//            case POST_TRANSFER:
//                if (1.0 < transferTimer.seconds() && transferTimer.seconds() < 1.2) {
////                    elevator.setElevatorState(Elevator.ElevatorState.LINES);
//                    outtake.outtakeState = Outtake.OuttakeState.SCORE;
//                }
//                if (stickyGamepad2.right_bumper) {
////                    elevator.setElevatorState(Elevator.ElevatorState.LINES);
//                    outtake.outtakeState  = Outtake.OuttakeState.SCORE;
//                }
//                break;
//            case SCORE:
//                if (stickyGamepad2.left_bumper) {
//                    outtake.rotateState = Outtake.RotateState.CENTER;
//                    outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//                    outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
////                    elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//                }
//
//                // SCORING OPTIONS
//                if (stickyGamepad2.a) {
//                    outtake.clawState = Outtake.ClawState.OPEN;
//                    scoreTimer.reset();
//                }
//                break;
//            case MANUAL:
//                if (stickyGamepad2.right_bumper) {
//                    transferTimer.reset();
//                }
//                if (0.25 < transferTimer.seconds() && transferTimer.seconds() < 0.5) {
//                    outtake.clawState = Outtake.ClawState.CLOSED;
//                }
//                if (0.55 < transferTimer.seconds() && transferTimer.seconds() < 0.8) {
//                    outtake.manualFourbarPos = Outtake.FOURBAR_POST_TRANSFER_POS;
//                }
//                if (0.9 < transferTimer.seconds() && transferTimer.seconds() < 1.15) {
//                    outtake.outtakeState = Outtake.OuttakeState.SCORE;
//                }
//
//                if (stickyGamepad2.a) {
//                    outtake.clawState = Outtake.ClawState.OPEN;
//                    scoreTimer.reset();
//                }
//
//                if (stickyGamepad2.left_bumper) {
//                    outtake.rotateState = Outtake.RotateState.CENTER;
//                    outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//                    outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
////                    elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//                }
//                break;
//        }
//        if (0.2 < scoreTimer.seconds() && scoreTimer.seconds() < 0.3) {
////            drive.setMotorPowers(-0.9, -0.9, -0.9, -0.9);
//            leaveBackboardTimer.reset();
//        }
//        if (0.45 < scoreTimer.seconds() && scoreTimer.seconds() < 0.55) {
//            outtake.rotateState = Outtake.RotateState.CENTER;
//            outtake.outtakeState = Outtake.OuttakeState.TRANSFER_PREP;
//            outtake.diffyHState = Outtake.DiffyHorizontalState.CENTER;
////            elevator.setElevatorState(Elevator.ElevatorState.TRANSFER);
//        }
//
//        // manual
//        if (gamepad2.left_stick_y != 0) {
//            outtake.outtakeState = Outtake.OuttakeState.MANUAL;
//            outtake.getManualValues();
//            outtake.manualHDiffy += gamepad2.left_stick_y * 0.005;
//        }
//        if (gamepad2.right_stick_y != 0) {
//            outtake.outtakeState = Outtake.OuttakeState.MANUAL;
//            outtake.getManualValues();
//            outtake.manualVDiffy += gamepad2.right_stick_y * -0.005;
//        }
//
//
//        telemetry.addData("outtake_state", outtake.outtakeState);
//        telemetry.addData("horizontal state", outtake.diffyHState);
//        telemetry.addData("diffy_horizontal", outtake.getDiffyHorizontal());
//        telemetry.addData("diffy_vertical", outtake.getDiffyVertical());
//        telemetry.addData("diffy H state", outtake.diffyHState);
//        telemetry.addData("rotate state", outtake.rotateState);
//        telemetry.addData("rotate value", outtake.getRotatePosition());
//        telemetry.addData("claw state", outtake.clawState);
//
//        telemetry.update();
//    }
//
//    private static String formatResults(MovingStatistics statistics) {
//        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
//                statistics.getMean() * 1000,
//                statistics.getStandardDeviation() * 1000,
//                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
//    }
//    @Override
//    public void stop() {
//
//    }
//}