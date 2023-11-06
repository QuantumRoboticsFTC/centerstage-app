package eu.qrobotics.centerstage.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.centerstage.teamcode.hardware.CachingDcMotorEx;

public class Elevator implements Subsystem {

    private CachingDcMotorEx motorLeft;
    private CachingDcMotorEx motorRight;
    private Robot robot;

    public Elevator(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderLeft"));
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "sliderRight"));

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        ;
    }
}
