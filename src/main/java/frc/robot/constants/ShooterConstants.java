package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class ShooterConstants {
    
    // Flywheel
    public static final int kLeftMotorID = 8;
    public static final int kRightMotorID = 9;
    public static final TalonFXInvertType kLeftMotorInversion = TalonFXInvertType.OpposeMaster;
    public static final TalonFXInvertType kRightMotorInversion = TalonFXInvertType.CounterClockwise;
    public static final double kMaxRPM = 4500;
    
    // Hood
    public static final int kLeftServoChannel = 0;
    public static final int kRightServoChannel = 1;
    public static final double kServoLengthMM = 100;
    public static final double kServoSpeedMM = 25;

    // Current limiting
    public static final int kContinuousCurrentLimit = 40;
    public static final int kPeakCurrentLimit = 60;
    public static final double kPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    // Feedforward
    public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
        0, // Voltage to break static friction
        0.02, // Volts per radian per second
        0.0025 // Volts per radian per second squared
    );

    // PID
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();

    static {
        kFlywheelConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        kFlywheelConfig.slot0.kP = kP;
        kFlywheelConfig.slot0.kI = kI;
        kFlywheelConfig.slot0.kD = kD;
        kFlywheelConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kContinuousCurrentLimit,
            kPeakCurrentLimit,
            kPeakCurrentDuration
        );
        kFlywheelConfig.voltageCompSaturation = kVoltageSaturation;
        kFlywheelConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
    }
}
