package eu.qrobotics.centerstage.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    private boolean reverse = true;
    MultipleTelemetry telem;

    private double cachedPower = 0;
    private double lastNonNullPower = 0;
    private double cachedPosition = 0;
    private double absolutePosition = 0;
    public double rotations = 0;
    private double epsilon = 4;

    public double diff = 0;

    public double iterationLimitUp = 45;
    public double iterationLimitDown = 20;

    private  List<String> changes = new ArrayList<String>();
    private  List<String> positions = new ArrayList<String>();

//    public AxonPlusServo(CRServo servo, AnalogInput encoder, MultipleTelemetry _telem) {
    public AxonPlusServo(CRServo servo, AnalogInput encoder) {
        delegateServo = servo;
        delegateEncoder = encoder;
        setAbsolutePosition(getRelativePosition());
//        telem = _telem;
    }

    public double getVoltage() {
        return delegateEncoder.getVoltage();
    }

    public double getRelativePosition() {
        return delegateEncoder.getVoltage() / 3.3 * 360;
    }

    public double getCachedPosition() {
        return cachedPosition;
    }

    public double getAbsolutePosition() {
        return absolutePosition;
    }

    public void setAbsolutePosition(double _absolutePosition) {
        absolutePosition = _absolutePosition;
        cachedPosition = _absolutePosition % 360;
    }

    public void update() {
        double newPosition = getRelativePosition();
        if (newPosition > cachedPosition + 3.0 ||
                newPosition < cachedPosition - 3.0) {
            changes.add("pos" + String.valueOf(newPosition));
        }

        if (lastNonNullPower < 0) {
            if ((0 <= 360 - cachedPosition + newPosition &&
                    360 - cachedPosition + newPosition <= iterationLimitUp) &&
                    newPosition + epsilon < cachedPosition) {
                rotations++;
                absolutePosition = absolutePosition +
                        (360 - cachedPosition + newPosition);
                diff = (360 - cachedPosition + newPosition);
                cachedPosition = newPosition;
            } else if ((0 <= newPosition - cachedPosition &&
                    newPosition - cachedPosition <= iterationLimitUp)) {
                absolutePosition = absolutePosition +
                        (newPosition - cachedPosition);
                diff = (newPosition - cachedPosition);
                cachedPosition = newPosition;
            }
        } else if (lastNonNullPower > 0) {
            if ((0 <= 360 - newPosition + cachedPosition &&
                    360 - newPosition + cachedPosition <= iterationLimitDown) &&
                    cachedPosition + epsilon < newPosition) {
                rotations--;
                absolutePosition = absolutePosition -
                        (360 - newPosition + cachedPosition);
                diff = (360 - newPosition + cachedPosition);
                cachedPosition = newPosition;
            } else if ((0 <= cachedPosition - newPosition &&
                    cachedPosition - newPosition <= iterationLimitDown)) {
                absolutePosition = absolutePosition -
                        (cachedPosition - newPosition);
                diff = cachedPosition - newPosition;
                cachedPosition = newPosition;
            }
        }
        if(changes.size()>=10){
            changes.remove(0);
        }
        if(positions.size()>=10){
            positions.remove(0);
        }

//        telem.addData("diff ", diff);
//        telem.addLine("Changes:");
//        for(String change:changes){
//            telem.addLine(change);
//        }
//        telem.addLine("cachedpos:");
//        for(String pos:positions){
//            telem.addLine(pos);
//        }
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

//    public  void reverseServo() {
//        reverse = true;
//    }

    @Override
    public void setPower(double power) {
        if (power != cachedPower) {
            if (reverse) {
                power = -power;
            }
            cachedPower = power;
            if (power != 0) {
                lastNonNullPower = power;
            }
            delegateServo.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return delegateServo.getPower();
    }

    public double getCachedPower() {
        return cachedPower;
    }

    public double getLastNonNullPower() {
        return lastNonNullPower;
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