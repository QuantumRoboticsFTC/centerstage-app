package eu.qrobotics.centerstage.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachingDcMotorEx extends CachingDcMotor implements DcMotorEx {
    private DcMotorEx delegate;

    public CachingDcMotorEx(DcMotorEx dcMotorEx) {
        super(dcMotorEx);
        delegate = dcMotorEx;
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        delegate.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        delegate.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return delegate.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        delegate.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        delegate.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        delegate.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return delegate.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return delegate.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return delegate.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        delegate.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return delegate.isOverCurrent();
    }
}