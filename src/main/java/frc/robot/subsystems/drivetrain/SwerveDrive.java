package frc.robot.subsystems.drivetrain;

import static frc.robot.auto.AutoConstants.*;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.AutoConstants;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule[] swerveMods;
    private final WPI_Pigeon2 gyro;
    private final BasePigeonSimCollection gyroSim; // simulate pigeon

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    // path controller and its dimension-specific controllers
    // i.e 1 meter error in the x direction = kP meters per second x velocity added
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // our auto rotation targets are profiled to obey velocity and acceleration constraints
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints
    );
    // our auto controller which follows trajectories and adjusts target chassis speeds to reach a desired pose
    private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController, thetaController);

    private final Field2d field2d = new Field2d();
    
    public SwerveDrive() {
        // construct our modules in order with their specific constants
        swerveMods = new SwerveModule[]{
            new SwerveModule(SwerveConstants.Module.FL),
            new SwerveModule(SwerveConstants.Module.FR),
            new SwerveModule(SwerveConstants.Module.BL),
            new SwerveModule(SwerveConstants.Module.BR)
        };

        gyro = new WPI_Pigeon2(SwerveConstants.kPigeon2ID);
        gyro.configAllSettings(SwerveConstants.kPigeon2Config);
        gyroSim = gyro.getSimCollection();
        zeroGyro();

        kinematics = new SwerveDriveKinematics(
            swerveMods[0].getModuleConstants().centerOffset,
            swerveMods[1].getModuleConstants().centerOffset,
            swerveMods[2].getModuleConstants().centerOffset,
            swerveMods[3].getModuleConstants().centerOffset
        );
        odometry = new SwerveDriveOdometry(kinematics, getGyroYaw());
        SmartDashboard.putData("Field", field2d);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(kThetaPositionTolerance, kThetaVelocityTolerance);
        pathController.setEnabled(true); // disable for feedforward-only auto
    }

    @Override
    public void periodic() {
        for(int i=0;i<4;i++){
            swerveMods[i].periodic();
        }

        // display our robot (and individual modules) pose on the field
        odometry.update(getGyroYaw(), getModuleStates());
        field2d.setRobotPose(getPose());
        field2d.getObject("Swerve Modules").setPoses(getModulePoses());
    }

    /**
     * Basic teleop drive control; percentages representing vx, vy, and omega
     * are converted to chassis speeds for the robot to follow
     * @param vxMeters vx (forward)
     * @param vyMeters vy (strafe)
     * @param omegaRadians omega (rotation CCW+)
     * @param fieldRelative If is field-relative control
     */
    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean openLoop, boolean fieldRelative){
        double vx = vxMeters;
        double vy = vyMeters;
        double omega = omegaRadians;
        ChassisSpeeds targetChassisSpeeds;
        if(fieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vx,vy,omega);
        }
        setChassisSpeeds(targetChassisSpeeds, openLoop, false);
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation){
        // determine ChassisSpeeds from path state and positional feedback control from HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
            getPose(),
            targetState,
            targetRotation
        );
        // command robot to reach the target ChassisSpeeds
        setChassisSpeeds(targetChassisSpeeds, false, false);
    }
    public boolean driveRotate(Rotation2d targetRotation, double vxMeters, double vyMeters, boolean fieldRelative){
        // rotation speed
        double rotationRadians = getPose().getRotation().getRadians();
        double pidOutput = thetaController.calculate(rotationRadians, targetRotation.getRadians());

        // + translation speed
        ChassisSpeeds targetChassisSpeeds;
        if(fieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, pidOutput, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vxMeters,vyMeters,pidOutput);
        }

        setChassisSpeeds(targetChassisSpeeds, false, true);
        return thetaController.atGoal();
    }
    public boolean driveRotate(Rotation2d targetRotation){
        return driveRotate(targetRotation, 0, 0, false);
    }

    /**
     * Command the swerve modules to the desired states.
     * Velocites above maximum speed will be downscaled (preserving ratios between modules)
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxLinearSpeed);
        for(int i=0;i<4;i++){
            swerveMods[i].setDesiredState(desiredStates[i], openLoop, steerInPlace);
        }
    }
    /**
     * Uses kinematics to convert ChassisSpeeds to module states.
     * @see {@link #setModuleStates(SwerveModuleState[], boolean)}
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
        setModuleStates(kinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
        this.targetChassisSpeeds = targetChassisSpeeds;
    }
    public void stop(){
        drive(0, 0, 0, true, false);
    }

    public void setBrakeOn(boolean is){
        for(SwerveModule mod : swerveMods){
            mod.setDriveBrake(is);
            mod.setSteerBrake(is);
        }
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getGyroYaw());
    }
    public void resetPathController(){
        xController.reset();
        yController.reset();
        thetaController.reset(getHeading().getRadians(), getChassisSpeeds().omegaRadiansPerSecond);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public Rotation2d getHeading(){
        return odometry.getPoseMeters().getRotation();
    }
    public Rotation2d getGyroYaw(){
        return gyro.getRotation2d();
    }

    public double getMaxLinearVelocityMeters(){
        return SwerveConstants.kMaxLinearSpeed;
    }
    public double getMaxAngularVelocityRadians(){
        return SwerveConstants.kMaxAngularSpeed;
    }
    
    /**
     * @return An ordered array filled with module states (rotation, velocity)
     */
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getAbsoluteState(),
            swerveMods[1].getAbsoluteState(),
            swerveMods[2].getAbsoluteState(),
            swerveMods[3].getAbsoluteState()
        };
    }
    /**
     * @return An ordered array filled with the module field poses 
     */
    public Pose2d[] getModulePoses(){
        Pose2d[] modulePoses = new Pose2d[4];
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            modulePoses[i] = getPose().transformBy(new Transform2d(module.getModuleConstants().centerOffset, module.getAbsoluteHeading()));
        }
        return modulePoses;
    }
    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public HolonomicDriveController getPathController(){
        return pathController;
    }

    public void log(){
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drive/Heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/X", pose.getX());
        SmartDashboard.putNumber("Drive/Y", pose.getY());
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        SmartDashboard.putNumber("Drive/VX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/VY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Omega Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            module.log();
        }
    }
    public void logTrajectory(Trajectory trajectory){
        field2d.getObject("Trajectory").setTrajectory(trajectory);
    }

    @Override
    public void simulationPeriodic(){
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            module.simulationPeriodic();
        }

        double chassisOmega = getChassisSpeeds().omegaRadiansPerSecond;
        chassisOmega = Math.toDegrees(chassisOmega);
        gyroSim.addHeading(chassisOmega*0.02);
    }

    public double getCurrentDraw(){
        double sum = 0;
        for(SwerveModule module : swerveMods) sum += module.getDriveCurrentDraw() + module.getSteerCurrentDraw();
        return sum;
    }
}
