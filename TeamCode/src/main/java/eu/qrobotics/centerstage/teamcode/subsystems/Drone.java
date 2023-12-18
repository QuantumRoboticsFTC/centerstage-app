package eu.qrobotics.centerstage.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Drone implements Subsystem {

    private Robot robot;

    public Drone(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
    }

    public static boolean IS_DISABLED = true;

    @Override
    public void update() {
        if (IS_DISABLED) return;
    }
}
