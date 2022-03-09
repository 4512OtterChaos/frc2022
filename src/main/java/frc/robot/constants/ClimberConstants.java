package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberConstants {
    public static final int kLeftMotorID = 12;
    public static final int kRightMotorID = 13;
    public static final boolean kMotorInverted = false;

    public static final double kTopHeightRotations = 140; // motor rotations
    public static final double kBottomHeightRotations = 0;

    public static final double kP = .25;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(55, 75);

    public static final int kContinuousCurrentLimit = 40;
    public static final int kPeakCurrentLimit = 50;
    public static final double kVoltageSaturation = 12;
    public static final int kCANTimeout = 100;
}
