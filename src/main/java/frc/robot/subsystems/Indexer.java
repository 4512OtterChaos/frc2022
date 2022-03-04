// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.constants.IndexerConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
    private final WPI_TalonFX motor = new WPI_TalonFX(kMotorID);
    private final DigitalInput bottomSensor = new DigitalInput(kBottomSensorID);
    private final DigitalInput topSensor = new DigitalInput(kTopSensorID);

    public Indexer() {
        setUpIndexer(true);
    }
    private void setUpIndexer(boolean init){
        if(init){
            motor.configAllSettings(kIndexerConfiguration);
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
    public boolean getBottomSensed(){
        return bottomSensor.get();
    }
    public boolean getTopSensed(){
        return topSensor.get();
    }
}
