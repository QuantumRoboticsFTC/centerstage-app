package eu.qrobotics.centerstage.teamcode.opmode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.centerstage.teamcode.hardware.OPColorSensor;

@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends LinearOpMode {

    private OPColorSensor sensor1;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensor1 = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "sensor1"));

        waitForStart();


        while (!isStopRequested()){

            multipleTelemetry.addData("Averaged Distance",sensor1.getDistance());
            multipleTelemetry.addData("Distance",sensor1.delegate.getDistance(DistanceUnit.MM));
            multipleTelemetry.update();

            sensor1.update();
        }

    }
}
