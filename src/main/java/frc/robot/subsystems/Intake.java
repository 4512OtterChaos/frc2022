// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    private final WPI_TalonFX motor = new WPI_TalonFX(kMotorID);
    
    // final DoubleSolenoid leftPiston = new DoubleSolenoid( PneumaticsModuleType.CTREPCM,kLeftPistonExtendPort, kLeftPistonRetractPort);
   // private final DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kRightPistonExtendPort, kRightPistonRetractPort);


    public Intake() {
        setUpIntake(true);
    }
    private void setUpIntake(boolean init){
        if(init){
            motor.configAllSettings(kIntakeConfiguration);
        }
        motor.setNeutralMode(NeutralMode.Brake);
        motor.enableVoltageCompensation(true);
        motor.setInverted(kMotorInverted);
        TalonUtil.configStatusNormal(motor);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public void setVoltage(double voltage){
        motor.set(voltage / kVoltageSaturation);
    }
    /*
    public void setExtended(boolean extended){
        DoubleSolenoid.Value value = extended ? Value.kForward : Value.kReverse;
        leftPiston.set(value);
        rightPiston.set(value);
    }
    */


    // Simulation
    private final TalonFXSimCollection flywheelMotorSim = new TalonFXSimCollection(motor);
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kFF.kv, kFF.ka),
        DCMotor.getFalcon500(1),
        1,
        VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(5))
    );

    @Override
    public void simulationPeriodic(){
        flywheelSim.setInputVoltage(motor.getMotorOutputVoltage());
        flywheelSim.update(0.02);

        double flywheelMotorVelocityNative = TalonUtil.radiansToVelocity(
            flywheelSim.getAngularVelocityRadPerSec(),
            1
        );
        flywheelMotorSim.setIntegratedSensorVelocity((int)flywheelMotorVelocityNative);
        flywheelMotorSim.setSupplyCurrent(flywheelSim.getCurrentDrawAmps()/2);

        flywheelMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public double getCurrentDraw(){
        return motor.getSupplyCurrent();
    }
}
