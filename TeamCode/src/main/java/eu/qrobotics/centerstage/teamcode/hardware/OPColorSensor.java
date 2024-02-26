package eu.qrobotics.centerstage.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OPColorSensor {
    private ColorRangeSensor delegate;
    private double averageDistance = 0;
    private double distanceSum = 0;
    private double entryCount = 0;

    private double timerLimit = 1.0;
    private ElapsedTime timer;

    public OPColorSensor(ColorRangeSensor sensor) {
        delegate = sensor;
    }

    public double getDistance() {
        return averageDistance;
    }

    public void update() {
        if (timer.seconds() < timerLimit) {
            distanceSum = distanceSum +
                    delegate.getDistance(DistanceUnit.MM);
            entryCount++;
        } else {
            averageDistance = distanceSum / entryCount;
            timer.reset();
            distanceSum = 0;
            entryCount = 0;
        }
    }

    public void close() {
        delegate.close();
    }
}