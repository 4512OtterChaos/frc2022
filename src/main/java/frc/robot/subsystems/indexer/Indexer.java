// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;



import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(kMotorID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private final DigitalInput bottomSensor = new DigitalInput(kBottomSensorID);
    private final DigitalInput topSensor = new DigitalInput(kTopSensorID);

    private final PIDController controller = new PIDController(kP, kI, kD);
    private double targetRPM = 0;

    private double lastPosition = 0;
    private double lastTime = Timer.getFPGATimestamp();
    private double estimatedRPM = 0; // we calculate velocity from position deltas to avoid latency

    public Indexer() {
        motor.setCANTimeout(kCANTimeout);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(kVoltageSaturation);
        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);

        lastPosition = encoder.getPosition();
    }

    @Override
    public void periodic() {
        estimateRPM();
        /*
        double targetVoltage = controller.calculate(estimatedRPM, targetRPM);
        motor.setVoltage(targetVoltage);
        */
    }
    public void setBrakeOn(boolean is){
        motor.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void stop(){
        setVoltage(0);
        setRPM(0);
    }
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    public void setVoltageIn(){setVoltage(kVoltageIn);}
    public void setVoltageFeed(){setVoltage(kVoltageFeed);}
    public void setVoltageOut(){setVoltage(kVoltageOut);}

    public void setRPM(double rpm){
        targetRPM = rpm;
    }

    /**
     * Estimate motor RPM by observing position deltas over time
     */
    private void estimateRPM(){
        double pos = encoder.getPosition();
        double time = Timer.getFPGATimestamp();

        estimatedRPM = (pos - lastPosition) / (time - lastTime) * 60;

        lastPosition = pos;
        lastTime = time;
    }
    
    public boolean getBottomSensed(){
        return !bottomSensor.get();
    }
    public boolean getTopSensed(){
        return !topSensor.get();
    }
    public boolean shouldIndex(){
        return getBottomSensed() && !getTopSensed();
    }

    public void log(){
        SmartDashboard.putBoolean("Indexer/Bottom Sensed", getBottomSensed());
        SmartDashboard.putBoolean("Indexer/Top Sensed", getTopSensed());
        SmartDashboard.putNumber("Indexer/RPM", estimatedRPM);
    }



    // Simulation
    // init
    {
        REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic(){
    }

    public double getCurrentDraw(){
        return motor.getOutputCurrent();
    }
}
