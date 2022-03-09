// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.constants.IndexerConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;
import static frc.robot.constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
    private final CANSparkMax motor = new CANSparkMax(kMotorID, MotorType.kBrushless);
    private final DigitalInput bottomSensor = new DigitalInput(kBottomSensorID);
    private final DigitalInput topSensor = new DigitalInput(kTopSensorID);
    private final PIDController controller = new PIDController(kP, kI, kD);
    private double targetRPM = 0;

    public Indexer() {
        motor.setCANTimeout(kCANTimeout);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(kVoltageSaturation);
        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double targetVoltage = controller.calculate(motor.getEncoder().getVelocity(), targetRPM);
        motor.setVoltage(targetVoltage);
    }
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    public void setRPM(double rpm){
        targetRPM = rpm;
        
    }
    
    public boolean getBottomSensed(){
        return !bottomSensor.get();
    }
    public boolean getTopSensed(){
        return !topSensor.get();
    }

    public void log(){
        SmartDashboard.putBoolean("Indexer/Bottom Sensed", getBottomSensed());
        SmartDashboard.putBoolean("Indexer/Top Sensed", getTopSensed());
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
