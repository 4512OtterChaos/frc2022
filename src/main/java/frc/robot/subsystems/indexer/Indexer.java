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
import edu.wpi.first.wpilibj.simulation.DIOSim;
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
    private boolean isManual = true;
    private double targetVoltage = 0;

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
        double adjustedVoltage = targetVoltage;

        if(!isManual){
            adjustedVoltage = controller.calculate(estimatedRPM, targetRPM) 
            + kFF.calculate(Units.rotationsPerMinuteToRadiansPerSecond(targetRPM));
        }
        motor.setVoltage(adjustedVoltage);

        
    }
    public void setBrakeOn(boolean is){
        motor.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void stop(){
        setVoltage(0);
    }
    public void setVoltage(double voltage){
        isManual = true;
        targetVoltage = voltage;
    }
    public void setVoltageIn(){setVoltage(kVoltageIn);}
    public void setVoltageFeed(){setVoltage(kVoltageFeed);}
    public void setVoltageOut(){setVoltage(kVoltageOut);}

    public void setRPM(double rpm){
        if(isManual){
            controller.reset();
        }
        isManual = false;
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
    /**
     * @return Estimated RPM of the indexer wheels
     */
    public double getRPM(){
        //return estimatedRPM;
        return encoder.getVelocity();
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
        SmartDashboard.putNumber("Indexer/RPM", getRPM());
    }



    // Simulation
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kFF.kv, kFF.ka),
        DCMotor.getNEO(1),
        1,
        VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(3))
    );
    private final DIOSim bottomSensorSim = new DIOSim(bottomSensor);
    private final DIOSim topSensorSim = new DIOSim(topSensor);
    {
        REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }
    @Override
    public void simulationPeriodic(){
        flywheelSim.setInputVoltage(motor.getAppliedOutput()*motor.getVoltageCompensationNominalVoltage());
        flywheelSim.update(0.02);

        double positionDelta = flywheelSim.getAngularVelocityRPM() / 60 * 0.02;
    }
    public void setBottomSimSensed(boolean is){bottomSensorSim.setValue(!is);}
    public void setTopSimSensed(boolean is){topSensorSim.setValue(!is);}

    public double getCurrentDraw(){
        return motor.getOutputCurrent();
    }
}
