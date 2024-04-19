package eu.qrobotics.centerstage.teamcode.cv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class AprilPoses {
    public static List<Pose2d> aprilPoses = Arrays.asList(new Pose2d(0, 0, 0), // dummy

            new Pose2d(60, -29.52, 0), // RED LEFT (4)
            new Pose2d(60, -35.43, 0), // RED MIDDLE (5)
            new Pose2d(60.23, -41.33, 0), // RED RIGHT (6)
            new Pose2d(60.23, 41.33, 0), // BLUE LEFT (1)
            new Pose2d(60, 35.43, 0), // BLUE MIDDLE (2)
            new Pose2d(60, 29.52, 0), // BLUE RIGHT (3)
            new Pose2d(-72, -42,Math.toDegrees(Math.PI)), // RED AUDIENCE LARGE (7)
            new Pose2d(-72, -36.4, Math.toDegrees(Math.PI)), // RED AUDIENCE SMALL (8)
            new Pose2d(-72, 36.4,Math.toDegrees(Math.PI)), // BLUE AUDIENCE SMALL (9)
            new Pose2d(-72, 42, Math.toDegrees(Math.PI)) // BLUE AUDIENCE LARGE (10)
    );
}