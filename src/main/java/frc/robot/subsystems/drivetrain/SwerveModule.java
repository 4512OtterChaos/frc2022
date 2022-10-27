package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveConstants.Module;
import frc.robot.util.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

@Log.Exclude
public class SwerveModule implements Loggable {

    // Module Constants
    private final Module moduleConstants;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();
    private double lastTargetTotalAngle = 0;

    // Hardware
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder steerEncoder;

    public SwerveModule(Module moduleConstants){
        this.moduleConstants = moduleConstants;

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new WPI_CANCoder(moduleConstants.cancoderID);

        setupDriveMotor(true);
        setupCancoder(true);
        setupSteerMotor(true);

        // Simulation
        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = steerMotor.getSimCollection();
        steerEncoderSim = steerEncoder.getSimCollection();
    }

    private void setupDriveMotor(boolean init){
        if(init){
            driveMotor.configAllSettings(driveConfig);
        }
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(kInvertDrive);
        TalonUtil.configStatusSolo(driveMotor);
        if(Robot.isSimulation()) TalonUtil.configStatusSim(driveMotor);
    }
    private void setupCancoder(boolean init){
        steerEncoder.configAllSettings(cancoderConfig);
        steerEncoder.configMagnetOffset(moduleConstants.angleOffset, 50);
    }
    private void setupSteerMotor(boolean init){
        if(init){
            steerMotor.configAllSettings(steerConfig);
        }
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setInverted(kInvertSteer);
        resetToAbsolute();
        TalonUtil.configStatusSolo(steerMotor);
        if(Robot.isSimulation()) TalonUtil.configStatusSim(steerMotor);
    }

    public void periodic(){
        // check if the motors had an oopsie, reapply settings
        if(driveMotor.hasResetOccurred()){
            setupDriveMotor(false);
        }
        if(steerMotor.hasResetOccurred()){
            setupSteerMotor(false);
        }
    }

    /**
     * Reset the steering motor integrated encoder to the position of the steering cancoder.
     * We want to use the integrated encoder for control, but need the absolute cancoder for determining our startup rotation.
     */
    public void resetToAbsolute(){
        double absolutePosition = TalonUtil.degreesToPosition(getAbsoluteHeading().getDegrees(), kSteerGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, boolean steerInPlace){
        Rotation2d currentRotation = getIntegratedHeading();
        // avoid turning more than 90 degrees by inverting speed on large angle changes
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        // our desired angle in [-pi, pi]
        double targetConstrainedAngle = desiredState.angle.getRadians();
        // our total current angle. This is not constrained to [-pi, pi]
        double currentTotalAngle = currentRotation.getRadians();
        // our current angle in [-pi, pi]
        double currentConstrainedAngle = MathUtil.angleModulus(currentTotalAngle);
        // convert our constrained target to the closest "total" angle near our current total
        double targetTotalAngle = currentTotalAngle + MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);

        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            targetTotalAngle = lastTargetTotalAngle;
            this.lastDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
            this.lastDesiredState = desiredState;
        }

        // convert our target radians to falcon position units
        double angleNative = TalonUtil.radiansToPosition(targetTotalAngle, kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        steerMotor.set(ControlMode.MotionMagic, angleNative);

        // convert our target meters per second to falcon velocity units
        double velocityNative = TalonUtil.metersToVelocity(
            desiredState.speedMetersPerSecond,
            kDriveGearRatio,
            kWheelCircumference
        );
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        double driveFFOutput = kDriveFF.calculate(this.lastDesiredState.speedMetersPerSecond)/kVoltageSaturation;
        if(!openLoop){
            driveMotor.set(
                ControlMode.Velocity, velocityNative, // Native falcon counts per 100ms
                DemandType.ArbitraryFeedForward, driveFFOutput // feedforward voltage to percent output
            );
        }
        else{
            driveMotor.set(driveFFOutput);
        }
    }

    public void setDriveBrake(boolean is){
        driveMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setSteerBrake(boolean is){
        steerMotor.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Module heading reported by steering motor integrated encoder.
     * <br></br>
     * NOT constrained to [-pi, pi]
     */
    public Rotation2d getIntegratedHeading(){
        return Rotation2d.fromDegrees(TalonUtil.positionToDegrees(steerMotor.getSelectedSensorPosition(), kSteerGearRatio));
    }
    /**
     * Module heading reported by steering cancoder
     */
    public Rotation2d getAbsoluteHeading(){
        return Rotation2d.fromDegrees(steerEncoder.getPosition()).plus(new Rotation2d());
    }

    /**
     * @return State describing integrated module rotation and velocity in meters per second
     */
    public SwerveModuleState getIntegratedState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getIntegratedHeading();
        return new SwerveModuleState(velocity, angle);
    }
    /**
     * @return State describing absolute module rotation and velocity in meters per second
     */
    public SwerveModuleState getAbsoluteState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getSelectedSensorVelocity(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getAbsoluteHeading();
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public Module getModuleConstants(){
        return moduleConstants;
    }
    public void configDriveSettings(TalonFXConfiguration driveConfig) {
        driveMotor.configAllSettings(driveConfig, 0);
    }
    public void configSteerSettings(TalonFXConfiguration steerConfig) {
        steerMotor.configAllSettings(steerConfig, 0);
    }

    static class SwerveModules implements Loggable {
        public final SwerveModule[] modules;
    
        public SwerveModules(SwerveModule... modules) {
            this.modules = modules;
        }

        public void each(Consumer<SwerveModule> func) {
            for(SwerveModule module : modules) {
                func.accept(module);
            }
        }
        public SwerveDriveKinematics getKinematics() {
            return new SwerveDriveKinematics(
                modules[0].getModuleConstants().centerOffset,
                modules[1].getModuleConstants().centerOffset,
                modules[2].getModuleConstants().centerOffset,
                modules[3].getModuleConstants().centerOffset
            );
        }
    
        @Config(defaultValueNumeric = SwerveConstants.kDriveKP)
        public void configDriveKP(double kP) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kP(0, kP);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kDriveKI)
        public void configDriveKI(double kI) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kI(0, kI);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kDriveKD)
        public void configDriveKD(double kD) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kD(0, kD);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kSteerKP)
        public void configSteerKP(double kP) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kP(0, kP);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kSteerKI)
        public void configSteerKI(double kI) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kI(0, kI);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kSteerKD)
        public void configSteerKD(double kD) {
            for(SwerveModule module : modules) {
                module.steerMotor.config_kD(0, kD);
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kSteerVelocity)
        public void configSteerVelocity(double velocity) {
            for(SwerveModule module : modules) {
                module.steerMotor.configMotionCruiseVelocity(
                    TalonUtil.rotationsToVelocity(velocity, SwerveConstants.kSteerGearRatio)
                );
            }
        }
        @Config(defaultValueNumeric = SwerveConstants.kSteerAcceleration)
        public void configSteerAccel(double accel) {
            for(SwerveModule module : modules) {
                module.steerMotor.configMotionAcceleration(
                    TalonUtil.rotationsToVelocity(accel, SwerveConstants.kSteerGearRatio)
                );
            }
        }
        @Config(defaultValueNumeric = 0.2)
        public void configSteerFF(double kv) {
            for(SwerveModule module : modules) {
                double kFF = kv / 12 * 1023 / TalonUtil.radiansToVelocity(1, 12.8);
                module.steerMotor.config_kF(0, kFF);
            }
        }
    }

    public void log(){
        SwerveModuleState state = getAbsoluteState();
        int num = moduleConstants.moduleNum;
        //SmartDashboard.putNumber("Module "+num+" Cancoder Degrees", getCancoderHeading().getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Target Degrees", lastDesiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Native", steerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Module "+num+"/Steer Target Native", steerMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("Module "+num+"/Steer Velocity Native", steerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Target Feet", Units.metersToFeet(lastDesiredState.speedMetersPerSecond));
    }


    // Simulation
    private final TalonFXSimCollection driveMotorSim;
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        kSteerGearRatio
    );
    private final CANCoderSimCollection steerEncoderSim;

    public void simulationPeriodic(){
        driveWheelSim.setInputVoltage(driveMotorSim.getMotorOutputLeadVoltage());
        steeringSim.setInputVoltage(steerMotorSim.getMotorOutputLeadVoltage());
        driveWheelSim.update(0.02);
        steeringSim.update(0.02);

        //SmartDashboard.putNumber("Drive Sim Model Amps", driveWheelSim.getCurrentDrawAmps());
        //SmartDashboard.putNumber("Drive Sim Model Velocity Feet", Units.metersToFeet(driveWheelSim.getAngularVelocityRPM() * kWheelCircumference / 60));
        double driveMotorVelocityNative = TalonUtil.rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, kDriveGearRatio);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));
        driveMotorSim.setSupplyCurrent(driveWheelSim.getCurrentDrawAmps()/2);

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steeringSim.getAngularVelocityRPM());
        double steerMotorVelocityNative = TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, kSteerGearRatio);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps()/2);
        
        steerEncoderSim.setVelocity((int)(TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        steerEncoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public double getDriveCurrentDraw(){
        return driveMotor.getSupplyCurrent();
    }
    public double getSteerCurrentDraw(){
        return steerMotor.getSupplyCurrent();
    }
}