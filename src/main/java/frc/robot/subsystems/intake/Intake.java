// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Intake extends SubsystemBase {

    private final WPI_TalonFX motor = new WPI_TalonFX(kMotorID);
    
    //private final DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kPistonExtendPort, kPistonRetractPort);

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
        TalonUtil.configStatusSlow(motor);
    }
    
    @Override
    public void periodic() {
    }
    public void stop(){
        setVoltage(0);
    }
    public void setBrakeOn(boolean is){
        motor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setVoltage(double voltage){
        motor.set(voltage / kVoltageSaturation);
    }
    public void setVoltageIn(){setVoltage(kVoltageIn);}
    public void setVoltageOut(){setVoltage(kVoltageOut);}
    /*
    public void setExtended(boolean extended){
        DoubleSolenoid.Value value = extended ? Value.kForward : Value.kReverse;
        pistons.set(value);
    }
    
    public boolean getExtended(){return pistons.get() == Value.kForward;}
    */
    /**
     * @return RPM of intake wheels
     */
    public double getRPM(){
        return TalonUtil.velocityToRotations(motor.getSelectedSensorVelocity(), 2) * 60;
    }

    public void log(){
        SmartDashboard.putNumber("Intake/RPM", getRPM());
    }


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
        flywheelSim.setInputVoltage(flywheelMotorSim.getMotorOutputLeadVoltage());
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
