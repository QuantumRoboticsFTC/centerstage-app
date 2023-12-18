package eu.qrobotics.centerstage.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachingCRServo implements CRServo {
    private CRServo delegate;
    private double cachedPosition = -10;

    public CachingCRServo(CRServo servo) {
        delegate = servo;
    }

    @Override
    public ServoController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public void setPower(double power) {
        delegate.setPower(power);
    }

    @Override
    public double getPower() {
        return delegate.getPower();
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegate.close();
    }
}