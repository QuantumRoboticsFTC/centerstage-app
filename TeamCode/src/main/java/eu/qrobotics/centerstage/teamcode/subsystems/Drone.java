package eu.qrobotics.centerstage.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drone implements Subsystem {

    private Robot robot;

    public Drone(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
    }

    @Override
    public void update() {
        ;
    }
}
