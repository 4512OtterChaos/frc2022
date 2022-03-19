package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public class Translation3d implements Interpolatable<Translation3d>{
    
    private final double x;
    private final double y;
    private final double z;

    public Translation3d() {
        this(0, 0, 0);
    }
    public Translation3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    /**
     * @param distance
     * @param yaw
     * @param pitch 0 on the xy-plane, upwards+
     */
    public Translation3d(double distance, Rotation2d yaw, Rotation2d pitch) {
        this.x = distance * yaw.getCos() * pitch.getCos();
        this.y = distance * yaw.getSin() * pitch.getCos();
        this.z = distance * pitch.getSin();
    }

    public double getX(){return x;}
    public double getY(){return y;}
    public Translation2d getXY(){return new Translation2d(x, y);}
    public double getZ(){return z;}
    public Translation2d getYZ(){return new Translation2d(y, z);}
    public Translation2d getXZ(){return new Translation2d(x, z);}

    public double getDistance(Translation3d other) {
        return Math.hypot(Math.hypot(other.x - x, other.y - y), other.z - z);
    }
    public double getNorm() {
        return Math.hypot(Math.hypot(x, y), z);
    }
    public Rotation2d getYawError(Translation3d other){
        return new Rotation2d(other.x - x, other.y - y);
    }
    public Rotation2d getPitchTan(Translation3d other){
        return new Rotation2d(getDistance(other), other.z - z);
    }

    public Translation3d plus(Translation3d other){
        return new Translation3d(other.x + x, other.y + y, other.z + z);
    }
    public Translation3d plusXY(Translation2d other){
        return new Translation3d(other.getX() + x, other.getY() + y, z);
    }
    public Translation3d plusZ(double z){
        return new Translation3d(x, y, this.z + z);
    }

    @Override
    public Translation3d interpolate(Translation3d endValue, double t) {
        return new Translation3d(
            MathUtil.interpolate(x, endValue.x, t),
            MathUtil.interpolate(y, endValue.y, t),
            MathUtil.interpolate(z, endValue.z, t)
        );
    }
}
