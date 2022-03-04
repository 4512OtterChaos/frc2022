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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
    private final CANSparkMax motor = new CANSparkMax(kMotorID, MotorType.kBrushless);
    private final DigitalInput bottomSensor = new DigitalInput(kBottomSensorID);
    private final DigitalInput topSensor = new DigitalInput(kTopSensorID);

    public Indexer() {
        motor.setCANTimeout(kCANTimeout);

        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(kVoltageSaturation);
        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    public boolean getBottomSensed(){
        return bottomSensor.get();
    }
    public boolean getTopSensed(){
        return topSensor.get();
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
