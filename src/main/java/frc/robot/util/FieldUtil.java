package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
    public static final Translation2d kFieldCenter = new Translation2d(Units.feetToMeters(54/2), Units.feetToMeters(27));
    public static Rotation2d getAngleTo(Translation2d from, Translation2d to){
        double x = to.getX() - from.getX();
        double y = to.getY() - from.getY();

        return new Rotation2d(Math.atan2(y, x));
    }
    public static Rotation2d getAngleToCenter(Translation2d from){
        return getAngleTo(from, kFieldCenter);
    }
}
