package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@TeleOp(name="ServoTester")
public class ServoTester extends LinearOpMode {

    private Servo servo;

    private StickyGamepad stickyGamepad;


    private double pos=0;

    @Override
    public void runOpMode() throws InterruptedException {

        servo=hardwareMap.get(Servo.class,"intakeServo");

        waitForStart();

        stickyGamepad=new StickyGamepad(gamepad1);

        pos=0.44;
        servo.setPosition(pos);

        while (!isStopRequested()){
            stickyGamepad.update();
            if(stickyGamepad.dpad_left){
                pos-=0.02;
                pos=Math.max(-1,pos);
            }
            if(stickyGamepad.dpad_right){
                pos+=0.02;
                pos=Math.min(1,pos);
            }
            if(stickyGamepad.dpad_up){
                servo.setPosition(pos);
            }
            telemetry.addData("Position",pos);
            telemetry.addData("State",gamepad1.dpad_up);
            telemetry.update();
        }

    }

    void setControls(){

    }
}
