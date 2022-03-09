package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IndexerConstants {
    public static final int kMotorID = 10;
    public static final int kBottomSensorID = 0;
    public static final int kTopSensorID = 1;
    
    public static final boolean kMotorInverted = true;

    // Feedforward loaded with cargo (simulation rn)
    public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
        0.75, // Voltage to break static friction
        0.05, // Volts per radian per second
        0.01 // Volts per radian per second squared
    );
    
    // Current limits
    public static final int kContinuousCurrentLimit = 30;
    public static final int kPeakCurrentLimit = 40;
    public static final double kPeakCurrentDuration = 0.1;
    // Voltage compensation`
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;
    public static final int kCANTimeout = 100;
}
