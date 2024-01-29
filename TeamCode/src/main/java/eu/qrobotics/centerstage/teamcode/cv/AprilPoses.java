package eu.qrobotics.centerstage.teamcode.cv;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class AprilPoses {
    public static List<Vector2d> aprilPoses = Arrays.asList(new Vector2d(0, 0), // dummy
            new Vector2d(60.23, 41.33), // BLUE LEFT (1)
            new Vector2d(60, 35.43), // BLUE MIDDLE (2)
            new Vector2d(60, 29.52), // BLUE RIGHT (3)
            new Vector2d(60, -29.52), // RED LEFT (4)
            new Vector2d(60, -35.43), // RED MIDDLE (5)
            new Vector2d(60.23, -41.33) // RED RIGHT (6)
    );
}
