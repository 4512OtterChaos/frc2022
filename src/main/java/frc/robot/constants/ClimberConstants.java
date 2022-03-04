package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberConstants {
    public static final int kLeftMotorID = 12;
    public static final int kRightMotorID = 13;

    public static final double kTopHeight = 100;
    public static final double kBottomHeight = 0;
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(0.15, 1);
    public static final double kContinuousCurrentLimit = 40;
    public static final double kPeakCurrentLimit = 50;
    public static final double kEncoderCountsTop = 0;
}
