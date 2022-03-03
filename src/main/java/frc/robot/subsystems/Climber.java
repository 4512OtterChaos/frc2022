// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final ProfiledPIDController leftController = new ProfiledPIDController(kP, kI, kD, kConstraints);
    private final ProfiledPIDController rightController = new ProfiledPIDController(kP, kI, kD, kConstraints);

    private boolean isManual = true;
    private double targetVolts = 0;
    private double targetHeight = 0;

    /** Creates a new Climber. */
    public Climber() {
        //set inversions idk?
        leftEncoder.setPositionConversionFactor(kTopHeight/kEncoderCountsTop);
        rightEncoder.setPositionConversionFactor(kTopHeight/kEncoderCountsTop);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double leftAdjustedVoltage = targetVolts;
        double rightAdjustedVoltage = targetVolts;

        

        if(!isManual){
            leftAdjustedVoltage = leftController.calculate(leftEncoder.getPosition(), targetHeight);
            rightAdjustedVoltage = rightController.calculate(rightEncoder.getPosition(), targetHeight);
        }
        SmartDashboard.putNumber("Left targetVolts", leftAdjustedVoltage);
        SmartDashboard.putNumber("Right targetVolts", rightAdjustedVoltage);


        if(leftEncoder.getPosition() >= kTopHeight) leftAdjustedVoltage = Math.min(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() >= kTopHeight) rightAdjustedVoltage = Math.min(rightAdjustedVoltage, 0);
        if(leftEncoder.getPosition() <= kBottomHeight) leftAdjustedVoltage = Math.max(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() <= kBottomHeight) rightAdjustedVoltage = Math.max(rightAdjustedVoltage, 0);

        rightMotor.setVoltage(rightAdjustedVoltage);
        leftMotor.setVoltage(leftAdjustedVoltage);
    }
    public void setClimberVolts(double voltage){
        isManual = true;
        targetVolts = voltage;
        SmartDashboard.putNumber("setVoltage", voltage);
    }

    public void setClimberHeight(double heightMeters){
        if(isManual){
            leftController.reset(leftEncoder.getPosition(), leftEncoder.getVelocity());
            rightController.reset(rightEncoder.getPosition(), rightEncoder.getVelocity());
        } 
        isManual = false;
        targetHeight = heightMeters;
    }
}
