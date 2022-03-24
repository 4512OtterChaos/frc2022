package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // Camera offset from robot center (intake +x axis)
    public static Transform2d kCameraOffset = new Transform2d(
        new Translation2d(Units.inchesToMeters(-12.5), 0),
        new Rotation2d(Math.PI)
    );
    public static double kCameraHeight = Units.inchesToMeters(26.5);
    public static Rotation2d kCameraPitch = Rotation2d.fromDegrees(37.5);
    // additional latency outside of pipeline
    public static double kLatencyMs = 20;
}
