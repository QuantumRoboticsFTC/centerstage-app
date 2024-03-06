package eu.qrobotics.centerstage.teamcode.hardware;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.centerstage.teamcode.util.Encoder;

public class AxonPlusServo implements CRServo {
    private CRServo delegateServo;
    public AnalogInput delegateEncoder;
    private boolean reverse=false;

    private double angleOffset=400;
    private double cachedPower = 0;
    private double cachedPosition = 0;
    public double rotations = 0;

    public double averageAngle = 0;
    private double angleSum = 0;
    private double entryCount = 0;

    private double timerLimit = 0.7;
    private ElapsedTime timer;



    public AxonPlusServo(CRServo servo, AnalogInput encoder) {
        delegateServo = servo;
        delegateEncoder = encoder;
        timer=new ElapsedTime();
    }

    public double getVoltage(){ return delegateEncoder.getVoltage();}
    public double getRelativePosition() {
        int pose=(int)(delegateEncoder.getVoltage() / 2.354 * 360);
        return pose;
    }

    public double getAbsolutePosition() {
        return rotations * 360 + cachedPosition-angleOffset;
    }

    public void update() {
        double newPosition = getRelativePosition();
        if(angleOffset==400){
            angleOffset=newPosition;
        }
        if (timer.seconds() < timerLimit) {
            angleSum = angleSum +
                    newPosition;
            entryCount++;
        } else {
            averageAngle = angleSum / entryCount;
            timer.reset();
            angleSum = 0;
            entryCount = 0;
        }
        if (cachedPower > 0) {
            if (averageAngle < cachedPosition) {
                rotations++;
            }
        } else if (cachedPower < 0) {
            if (cachedPosition < averageAngle) {
                rotations--;
            }
        }
        cachedPosition =averageAngle;
    }

    @Override
    public ServoController getController() {
        return delegateServo.getController();
    }

    @Override
    public int getPortNumber() {
        return delegateServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        delegateServo.setDirection(direction);
    }

    public  void reverseServo(){
        reverse=true;
    }

    @Override
    public void setPower(double power) {
        if (power != cachedPower) {
            cachedPower = power;
            if(reverse){
                power=-power;
            }
            delegateServo.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return delegateServo.getPower();
    }

    @Override
    public Direction getDirection() {
        return delegateServo.getDirection();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegateServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegateServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegateServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegateServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegateServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegateServo.close();
    }
}