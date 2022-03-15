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
