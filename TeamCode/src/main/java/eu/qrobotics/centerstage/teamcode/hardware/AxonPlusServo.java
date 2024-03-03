package eu.qrobotics.centerstage.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class AxonPlusServo implements CRServo {
    private CRServo delegateServo;
    private AnalogInput delegateEncoder;
    private double cachedPower = 0;
    private double cachedPosition = 0;
    private double rotations = 0;

    public AxonPlusServo(CRServo servo, AnalogInput encoder) {
        delegateServo = servo;
        delegateEncoder = encoder;
    }

    public double getRelativePosition() {
        return delegateEncoder.getVoltage() / 3.3 * 360;
    }

    public double getAbsolutePosition() {
        return rotations * 360 + cachedPosition;
    }

    public void update() {
        double newPosition = getRelativePosition();
        if (cachedPower > 0) {
            if (newPosition < cachedPosition) {
                rotations++;
            }
        } else if (cachedPower < 0) {
            if (cachedPosition < newPosition) {
                rotations--;
            }
        }
        cachedPosition = newPosition;
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

    @Override
    public void setPower(double power) {
        if (power != cachedPower) {
            delegateServo.setPower(power);
            cachedPower = power;
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