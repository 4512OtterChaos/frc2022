package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class AutoConstants {
    
    // our maximum speeds/accelerations during auto
    public static final double kMaxLinearSpeed = Units.feetToMeters(10);
    public static final double kMaxLinearAcceleration = Units.feetToMeters(12);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(600);
    public static final double kMaxAngularAcceleration = Units.degreesToRadians(1080);

    public static final double kPXController = 3; // pose PID control. 1 meter error in x = kP meters per second in target x velocity 
    public static final double kPYController = 3;
    public static final double kPThetaController = 4;
    // constraints for the theta controller on velocity (omega) and acceleration (alpha)
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAcceleration
    );

    // packaged configs for path following
    public static final TrajectoryConfig kMaxSpeedConfig = new TrajectoryConfig(kMaxLinearSpeed, kMaxLinearAcceleration);
    public static final TrajectoryConfig kMediumSpeedConfig = new TrajectoryConfig(0.6*kMaxLinearSpeed, 0.6*kMaxLinearAcceleration);
    public static final TrajectoryConfig kSlowSpeedConfig = new TrajectoryConfig(0.3*kMaxLinearSpeed, 0.3*kMaxLinearAcceleration);
}
