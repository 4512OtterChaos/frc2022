package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class IndexerConstants {
    public static final int kMotorID = 10;
    public static final int kBottomSensorID = 0;
    public static final int kTopSensorID = 1;
    
    public static final boolean kMotorInverted = false;
    public static final TalonFXConfiguration kIndexerConfiguration = new TalonFXConfiguration();
    public static final int kIndexerContinuousCurrentLimit = 30;
    public static final int kIndexerPeakCurrentLimit = 40;
    public static final double kIndexerPeakCurrentDuration = 100;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    static {
        kIndexerConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        //IndexerConfiguration.slot0.kP = kDriveKP;
        //IndexerConfiguration.slot0.kI = kDriveKI;
        //IndexerConfiguration.slot0.kD = kDriveKD;
        kIndexerConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kIndexerContinuousCurrentLimit,
            kIndexerPeakCurrentLimit,
            kIndexerPeakCurrentDuration
        );
        kIndexerConfiguration.voltageCompSaturation = kVoltageSaturation;
        kIndexerConfiguration.voltageMeasurementFilter = kVoltageMeasurementSamples;
    }
}
