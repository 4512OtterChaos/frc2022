package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
    public static final Translation2d kFieldCenter = new Translation2d(
        Units.feetToMeters(54.0/2),
        Units.feetToMeters(27.0/2)
    );
    // distance from the center of the hub to the fender wall
    public static final double kCenterToFenderDist = Units.inchesToMeters(33.512);
    public static final double kVisionRingDiameter = Units.inchesToMeters(53.375);
    // ground to center of the vision tape
    public static final double kVisionRingHeight = Units.inchesToMeters(102.625);

    public static final double kUpperGoalBottomDiameter = Units.inchesToMeters(16.75);
    // Bottom of the upper hub (agitator) from the ground
    public static final double kUpperGoalBottomHeight = Units.inchesToMeters(73);
    // upper hub slope
    public static final Rotation2d kUpperHubIncline = new Rotation2d(
        (kVisionRingDiameter - kUpperGoalBottomDiameter)/2.0,
        kVisionRingHeight - kUpperGoalBottomHeight
    );
    // diameter of the circle that the cargo adjacent to the tarmac is placed on
    public static final double kCargoPlacementDiameter = Units.inchesToMeters(306);
    
    public static Rotation2d getAngleTo(Translation2d from, Translation2d to){
        double x = to.getX() - from.getX();
        double y = to.getY() - from.getY();

        return new Rotation2d(Math.atan2(y, x));
    }
    public static Rotation2d getAngleToCenter(Translation2d from){
        return getAngleTo(from, kFieldCenter);
    }
}
