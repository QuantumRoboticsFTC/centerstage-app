package eu.qrobotics.centerstage.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OPDistanceSensor {
    public DistanceSensor delegate;
    private double averageDistance = 0;
    private double distanceSum = 0;
    private double entryCount = 0;

    private double timerLimit = 0.3;
    private ElapsedTime timer;

    public OPDistanceSensor(DistanceSensor sensor) {
        delegate = sensor;
        timer = new ElapsedTime();
    }

    public double getDistance() {
        return averageDistance;
    }

    public void update() {
        averageDistance = delegate.getDistance(DistanceUnit.MM);
//        if (timer.seconds() < timerLimit) {
//            distanceSum = distanceSum +
//                    delegate.getDistance(DistanceUnit.MM);
//            entryCount++;
//        } else {
//            averageDistance = distanceSum / entryCount;
//            timer.reset();
//            distanceSum = 0;
//            entryCount = 0;
//        }
    }

    public void close() {
        delegate.close();
    }
}