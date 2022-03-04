package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class IntakeConstants {
    public static final int kMotorID = 11;
    public static final int kLeftPistonExtend = 0;
    public static final int kRightPistonExtend = 1;
    public static final int kLeftPistonRetract = 2;
    public static final int kRightPistonRetract = 3;
    public static final boolean kMotorInverted = false;
    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration();

    public static final int kIntakeContinuousCurrentLimit = 30;
    public static final int kIntakePeakCurrentLimit = 40;
    public static final double kIntakePeakCurrentDuration = 100;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    static {
        kIntakeConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        //intakeConfiguration.slot0.kP = kDriveKP;
        //intakeConfiguration.slot0.kI = kDriveKI;
        //intakeConfiguration.slot0.kD = kDriveKD;
        kIntakeConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kIntakeContinuousCurrentLimit,
            kIntakePeakCurrentLimit,
            kIntakePeakCurrentDuration
        );
        kIntakeConfiguration.voltageCompSaturation = kVoltageSaturation;
        kIntakeConfiguration.voltageMeasurementFilter = kVoltageMeasurementSamples;
    }
}
