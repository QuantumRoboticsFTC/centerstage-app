package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import eu.qrobotics.centerstage.teamcode.hardware.CachingServo;
import eu.qrobotics.centerstage.teamcode.util.StickyGamepad;

@Config
@TeleOp(name="ServoPoseConfig")
public class ServoPoseConfig extends LinearOpMode {

    private CachingServo servo;
    private StickyGamepad stickyGamepad;
    public static double pose=0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = new CachingServo(hardwareMap.get(ServoImplEx.class, "outtakeRotate"));
        stickyGamepad=new StickyGamepad(gamepad1);

        waitForStart();

        while (!isStopRequested()){

            if(stickyGamepad.dpad_up){
                pose+=0.05;
                pose=Math.min(pose,1.0);
            }
            if(stickyGamepad.dpad_down){
                pose-=0.05;
                pose=Math.max(pose,-1.0);
            }

            servo.setPosition(pose);

            multipleTelemetry.addData("Current Pose",servo.getPosition());
            multipleTelemetry.addData("Pose",pose);
            multipleTelemetry.update();
            stickyGamepad.update();
        }


    }
}
