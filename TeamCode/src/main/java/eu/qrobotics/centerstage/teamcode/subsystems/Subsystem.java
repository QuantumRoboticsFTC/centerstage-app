package eu.qrobotics.centerstage.teamcode.subsystems;

public interface Subsystem {
    /**
     * Run control code (e.g., read sensors and update motors)
     * TODO: Return telemetry.
     */
    void update();

    default void stop() { }
}