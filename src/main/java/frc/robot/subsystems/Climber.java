// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(kLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(kRightMotorID, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final ProfiledPIDController leftController = new ProfiledPIDController(kP, kI, kD, kConstraints);
    private final ProfiledPIDController rightController = new ProfiledPIDController(kP, kI, kD, kConstraints);

    private boolean isManual = true;
    private double targetVolts = 0;
    private double targetRotations = 0;

    /** Creates a new Climber. */
    public Climber() {
        leftMotor.setCANTimeout(kCANTimeout);
        rightMotor.setCANTimeout(kCANTimeout);

        leftMotor.setInverted(kMotorInverted);
        rightMotor.setInverted(kMotorInverted);
        leftMotor.enableVoltageCompensation(kVoltageSaturation);
        rightMotor.enableVoltageCompensation(kVoltageSaturation);
        leftMotor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);
        rightMotor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double leftAdjustedVoltage = targetVolts;
        double rightAdjustedVoltage = targetVolts;

        if(!isManual){
            leftAdjustedVoltage = leftController.calculate(leftEncoder.getPosition(), targetRotations);
            rightAdjustedVoltage = rightController.calculate(rightEncoder.getPosition(), targetRotations);
        }

        if(leftEncoder.getPosition() >= kTopHeightRotations) leftAdjustedVoltage = Math.min(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() >= kTopHeightRotations) rightAdjustedVoltage = Math.min(rightAdjustedVoltage, 0);
        if(leftEncoder.getPosition() <= kBottomHeightRotations) leftAdjustedVoltage = Math.max(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() <= kBottomHeightRotations) rightAdjustedVoltage = Math.max(rightAdjustedVoltage, 0);

        rightMotor.setVoltage(rightAdjustedVoltage);
        leftMotor.setVoltage(leftAdjustedVoltage);
    }
    public void setVolts(double voltage){
        isManual = true;
        targetVolts = voltage;
    }

    public void setRotations(double rotations){
        if(isManual){
            leftController.reset(leftEncoder.getPosition(), leftEncoder.getVelocity());
            rightController.reset(rightEncoder.getPosition(), rightEncoder.getVelocity());
        } 
        isManual = false;
        targetRotations = rotations;
    }
    public void setTopHeightRotations(){setRotations(kTopHeightRotations);}
    public void setBottomHeighRotations(){setRotations(kBottomHeightRotations);}

    public void log(){
        SmartDashboard.putNumber("Climber/Left Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("Climber/Right Pos", rightEncoder.getPosition());
    }



    // simulation init
    {
        REVPhysicsSim.getInstance().addSparkMax(leftMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(rightMotor, DCMotor.getNEO(1));
    }
    @Override
    public void simulationPeriodic() {
    }

    public double getCurrentDraw(){
        return leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent();
    }
}
