package frc.robot.simulation;

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
    public double getZ(){return z;}
    public Translation2d getXY(){return new Translation2d(x, y);}
    public Translation2d getYZ(){return new Translation2d(y, z);}
    public Translation2d getXZ(){return new Translation2d(x, z);}
    
    public double getDistance() {
        return Math.hypot(Math.hypot(x, y), z);
    }
    public double getDistance(Translation3d other) {
        return other.minus(this).getDistance();
    }
    public Translation3d getNormalized(){
        return scale(1.0 / getDistance());
    }
    public Rotation2d getYaw(){
        return new Rotation2d(x, y);
    }
    public Rotation2d getYaw(Translation3d other){
        return other.minus(this).getYaw();
    }
    public Rotation2d getPitch(){
        return new Rotation2d(getXY().getNorm(), z);
    }
    public Rotation2d getPitch(Translation3d other){
        return other.minus(this).getPitch();
    }
    public Rotation2d getAngleBetween(Translation3d other){
        return new Rotation2d(Math.acos(dot(other) / (getDistance() * other.getDistance())));
    }
    
    public Translation3d plus(Translation3d other){
        return new Translation3d(other.x + x, other.y + y, other.z + z);
    }
    public Translation3d minus(Translation3d other){
        return new Translation3d(x - other.x, y - other.y, z - other.z);
    }
    public Translation3d plusXY(Translation2d other){
        return new Translation3d(other.getX() + x, other.getY() + y, z);
    }
    public Translation3d minusXY(Translation2d other){
        return new Translation3d(x - other.getX(), y - other.getY(), z);
    }
    public Translation3d plusX(double x){
        return new Translation3d(this.x + x, y, z);
    }
    public Translation3d plusY(double y){
        return new Translation3d(x, this.y + y, z);
    }
    public Translation3d plusZ(double z){
        return new Translation3d(x, y, this.z + z);
    }
    
    public Translation3d scale(double scalar){
        return new Translation3d(x*scalar, y*scalar, z*scalar);
    }
    public Translation3d unaryMinus(){
        return scale(-1);
    }
    
    public double dot(Translation3d other){
        return x*other.x + y*other.y + z*other.z;
    }
    public Translation3d proj(Translation3d other){
        if(Double.isNaN(other.getDistance())) return this;
        return other.scale(dot(other) / other.dot(other));
    }
    public Translation3d orth(Translation3d other){
        return minus(proj(other));
    }
    
    public Translation3d rotateByYaw(Rotation2d yaw){
        return new Translation3d(
        getDistance(),
        getYaw().plus(yaw),
        getPitch()
        );
    }
    public Translation3d rotateByPitch(Rotation2d pitch){
        return new Translation3d(
        getDistance(),
        getYaw(),
        getPitch().plus(pitch)
        );
    }
    
    @Override
    public String toString() {
        return String.format("Translation3d(X: %.2f, Y: %.2f, Z: %.2f)", x, y, z);
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
