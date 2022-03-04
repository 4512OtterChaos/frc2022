// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    private final WPI_TalonFX motor = new WPI_TalonFX(kMotorID);
    
    private final DoubleSolenoid leftPiston = new DoubleSolenoid( PneumaticsModuleType.CTREPCM,kLeftPistonExtend, kLeftPistonRetract);
    private final DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kRightPistonExtend, kRightPistonRetract);


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
        motor.setVoltage(voltage);
    }
    public void setExtended(boolean extended){
        DoubleSolenoid.Value value = extended ? Value.kForward : Value.kReverse;
        leftPiston.set(value);
        rightPiston.set(value);
    }
}
