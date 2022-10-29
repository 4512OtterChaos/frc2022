package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class CargoSimPhysics {
    public static final double kGravityAccel = -9.8;
    public static final double kMass = 0.27; // kg
    public static final double kRadius = Units.inchesToMeters(9.5 / 2.0);

    public static final double kIntakeOffset = Units.inchesToMeters(20);
    public static final double kIntakeHeight = Units.inchesToMeters(kRadius);
    // cargo closer than this will become intaked
    public static final double kIntakeXDist = kRadius + Units.inchesToMeters(2);
    public static final double kIntakeYDist = 2*kRadius + Units.inchesToMeters(2);

    // indexer waypoints
    public static final double kBotSensorBeginDistance = Units.inchesToMeters(11);
    public static final double kIndexBeginDistance = Units.inchesToMeters(16); // 25% intake, 75% index
    public static final double kBotSensorEndDistance = Units.inchesToMeters(22);
    public static final double kTopSensorBeginDistance = Units.inchesToMeters(34);
    // distance from intake entrance to shooter exit
    public static final double kPathDistance = Units.inchesToMeters(37);

    public static final double kShooterHeight = Units.inchesToMeters(24);

    public static final double kFloorBounceFactor = 0.75; // percent velocity retained
    public static final double kHubBounceFactor = 0.5; // percent velocity retained
    public static final double kMinBounceVel = 0.5; // velocities less than this will not bounce
    public static final double kFloorBounceVelFriction = 0.1; // additional m/s loss on floor bounce
    public static final double kHubBounceVelFriction = 0.2; // additional m/s loss on hub bounce
    public static final double kStaticFrictionAccel = 0.4; // m/s/s deceleration while rolling
    public static final double kSpinVel = 0.4; // added velocity from spin on first contact

    public static double calcBounceVel(
            double currVelIntoBounce, double bounceFactor, double bounceFrictionVel
    ){
        if(Math.abs(currVelIntoBounce) < kMinBounceVel) return 0;
        if(currVelIntoBounce < 0){
            return Math.max(0, -currVelIntoBounce*bounceFactor - bounceFrictionVel);
        }
        else{
            return Math.min(0, -currVelIntoBounce*bounceFactor + bounceFrictionVel);
        } 
    }
    public static double calcRollVel(double velocity, double rollFrictionAccel, double dt){
        if(velocity >= 0){
            return Math.max(0, velocity - rollFrictionAccel*dt);
        }
        else{
            return Math.min(0, velocity + rollFrictionAccel*dt);
        }
    }
    public static double calcAirDragVel(double velocity, double dt){
        double dragArea = 2 * Math.PI * kRadius*kRadius;
        double dragCoefficient = 0.2;
        double airDensity = 1.2;

        double force = 0.5 * dragCoefficient * airDensity * (velocity*velocity) * dragArea;

        return velocity - Math.copySign((force / kMass * dt), velocity);
    }

    /**
     * Calculate velocities after colliding with a field surface.
     * @param velocities vx, vy, vz
     * @param spinDirection Direction spin velocity will be applied
     * @param spinVel Velocity to be applied in spin direction
     * @param surfaceNormal Normal vector of the plane defining the contacted surface
     * @param bounceFactor Percentage bounce restitution
     * @param bounceFrictionVel Velocity lost along bounce vector (vel normal to surface)
     * @param rollFrictionAccel Acceleration due to rolling friction (vel along surface)
     * @param dt
     * @return Velocities after collision
     */
    public static Translation3d collideOnField(
            Translation3d velocities, Rotation2d spinDirection, double spinVel,
            Translation3d surfaceNormal,
            double bounceFactor, double bounceFrictionVel, double rollFrictionAccel,
            double dt
    ){
        Translation3d bounceVector = velocities.proj(surfaceNormal).unaryMinus();
        double bounceVelocity = calcBounceVel(
            -bounceVector.getDistance(),
            bounceFactor,
            bounceFrictionVel
        );
        
        Translation3d rollVector = velocities.plus(bounceVector);
        double rollVelocity = calcRollVel(
            rollVector.getDistance(),
            rollFrictionAccel,
            dt
        );
        
        if(bounceVector.getDistance() != 0){
            bounceVector = bounceVector.scale(bounceVelocity / bounceVector.getDistance());
        }
        if(rollVector.getDistance() != 0){
            rollVector = rollVector.scale(rollVelocity / rollVector.getDistance());
        }
        
        // recombine roll/bounce into new cargo velocities
        velocities = rollVector.plus(bounceVector);
        if(spinVel != 0){
            velocities = velocities.plus(
                new Translation3d(
                    spinVel,
                    spinDirection,
                    surfaceNormal.getPitch().minus(new Rotation2d(Math.PI/2.0))
                )
            );
        }

        return velocities;
    }

    public static Translation3d calcShotVelocities(
            double shotVelocity,
            Rotation2d shotPitch,
            Rotation2d shotYaw,
            ChassisSpeeds robotSpeeds
    ){
        return new Translation3d(
            shotVelocity,
            shotYaw,
            shotPitch
        )
        // transferred robot velocities
        .plusXY(new Translation2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond
        ).rotateBy(shotYaw.plus(new Rotation2d(Math.PI))));
    }
}
