package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.centerstage.teamcode.subsystems.Intake;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@Config
@TeleOp(name="IntakePos")
public class IntakePos extends LinearOpMode {

    public Robot robot;
    public StickyGamepad stickyGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.intake.dropdownState = Intake.DropdownState.STACK_5;
        stickyGamepad=new StickyGamepad(gamepad1);

        robot.start();

        while (!isStopRequested()){
            stickyGamepad.update();

            if(gamepad1.dpad_down){
                robot.intake.dropdownState=robot.intake.dropdownState.next();
            }
            if(gamepad1.dpad_up){
                robot.intake.dropdownState=robot.intake.dropdownState.previous();
            }

            switch (robot.intake.intakeMode) {
                case IN:
                    if (stickyGamepad.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    }
                    if (stickyGamepad.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    }
                    break;
                case IDLE:
                    if (stickyGamepad.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.OUT;
                    }
                    if (stickyGamepad.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    }
                    break;
                case OUT:
                    if (stickyGamepad.left_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IN;
                    }
                    if (stickyGamepad.right_bumper) {
                        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                    }
                    break;
            }


            telemetry.addLine( robot.intake.dropdownState.name());
            telemetry.update();
        }
        robot.stop();

    }
}
