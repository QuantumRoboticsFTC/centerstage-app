package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "ImuAcce")
public class ImuAcce extends LinearOpMode {

    private IMU imuEhub;

    public static int windowSize=10;

    public static double a=0.7;
    private double prevEstimate=0;

    private List<Double> window=new ArrayList<Double>();

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU.Parameters IMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imuEhub = hardwareMap.get(IMU.class, "bbno$");
        imuEhub.initialize(IMUParameters);


        for(int i=0;i<windowSize;i++){
            window.add(0.0);
        }


        waitForStart();

        while (!isStopRequested()){
            if(windowSize%2==0){
                windowSize++;
            }
            double acceleration=imuEhub.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            window.remove(0);
            window.add(acceleration);
            for(int i=window.size();i<windowSize;i++){
                window.add(0.0);
            }
            window.sort((v1, v2)->(v1<v2?1:0));
            for(int i=window.size();i>windowSize;i--){
                window.remove(0);
            }

            double estimate=a*acceleration+(1-a)*prevEstimate;
            prevEstimate=estimate;

            multipleTelemetry.addData("Angle ",String.valueOf(imuEhub.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            multipleTelemetry.addData("Z acceleration ",acceleration);
            multipleTelemetry.addData("Median filtered Z acceleration ", window.get(windowSize/2));
            multipleTelemetry.addData("Low pass filtered Z acceleration ", estimate);
            multipleTelemetry.update();
        }

    }
}
