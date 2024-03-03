package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@Config
@TeleOp(name = "ServoTester")
public class ServoTester extends LinearOpMode {

    private CRServo servo;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(2, 0, 0);
    private PIDFController pidfController = new PIDFController(pidCoefficients);
    private AnalogInput encoder;

    private StickyGamepad stickyGamepad;

    private double currentPos = 0;
    private double prevPos = 0;
    private double absolutePos = 0;
    public static double targetPos = 0;

    private double range=360;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(CRServo.class, "intakeServo");
        encoder = hardwareMap.get(AnalogInput.class, "intakeEncoder");

        waitForStart();


        while (!isStopRequested()) {

            currentPos=encoder.getVoltage()/3.3*range;
            absolutePos+=getPositionDiff();
            prevPos=currentPos;

            pidfController.setTargetPosition(targetPos);
            servo.setPower(pidfController.update(absolutePos));

            telemetry.addData("Encoder position", currentPos);
            telemetry.addData("Absolute position", absolutePos);
            telemetry.addData("Target position", targetPos);
            telemetry.update();
        }

    }

    private double getPositionDiff(){
        if(prevPos>currentPos){
            double dist1= Math.abs(prevPos-currentPos);
            double dist2=Math.abs(range-prevPos)+ Math.abs(currentPos);
            if(dist1<dist2){
                return dist1;
            }
            else{
                return  -dist2;
            }
        }
        else {
            double dist1= Math.abs(prevPos-currentPos);
            double dist2=Math.abs(range-currentPos)+ Math.abs(prevPos);
            if(dist1<dist2){
                return -dist1;
            }
            else{
                return  dist2;
            }
        }
    }
}
