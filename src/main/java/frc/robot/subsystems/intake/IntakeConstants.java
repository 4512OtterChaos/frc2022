package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeConstants {
    public static final int kMotorID = 11;
    public static final int kLeftPistonExtendPort = 0;
    public static final int kRightPistonExtendPort = 1;
    public static final int kLeftPistonRetractPort = 2;
    public static final int kRightPistonRetractPort = 3;

    public static final boolean kMotorInverted = true;

    // Feedforward (simulation rn)
    public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
        0.2, // Voltage to break static friction
        0.025, // Volts per radian per second
        0.0025 // Volts per radian per second squared
    );
    
    // Current limits
    public static final int kContinuousCurrentLimit = 30;
    public static final int kPeakCurrentLimit = 40;
    public static final double kPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration();

    static {
        kIntakeConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        //intakeConfiguration.slot0.kP = kDriveKP;
        //intakeConfiguration.slot0.kI = kDriveKI;
        //intakeConfiguration.slot0.kD = kDriveKD;
        kIntakeConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kContinuousCurrentLimit,
            kPeakCurrentLimit,
            kPeakCurrentDuration
        );
        kIntakeConfiguration.voltageCompSaturation = kVoltageSaturation;
        kIntakeConfiguration.voltageMeasurementFilter = kVoltageMeasurementSamples;
    }
}
