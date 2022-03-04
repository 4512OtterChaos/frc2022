package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.common.ShotMap;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final ShotMap shotMap = new ShotMap(shooter);
    private final Superstructure superstructure = new Superstructure(climber, drivetrain, indexer, intake, shooter, shotMap);

    private final OCXboxController controller = new OCXboxController(0);

    private final OCXboxController driver = new OCXboxController(0);
    private boolean isFieldRelative = true;

    private final AutoOptions autoOptions = new AutoOptions(drivetrain);

    public RobotContainer(){

        configureDriverBinds();

        autoOptions.submit();
    }

    public void periodic(){
    }

    public Command getAutoCommand(){
        return autoOptions.getSelected();
    }

    public void disable(){
        drivetrain.drive(0, 0, 0, false, false);
    }

    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
    }

    private void configureDriverBinds(){
        // when no other command is using the drivetrain, we
        // pass the joysticks for forward, strafe, and angular control
        Command teleopDrive = new RunCommand(()->{
            drivetrain.drive(
                driver.getForward(),
                driver.getStrafe(),
                driver.getTurn(),
                true,
                isFieldRelative);
        }, drivetrain).beforeStarting(()->driver.resetLimiters());
        drivetrain.setDefaultCommand(teleopDrive);

        // push-to-change driving "speed"
        driver.rightBumper
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));

        // change from field-relative to robot-relative control
        driver.backButton.whenPressed(()->{
            isFieldRelative = !isFieldRelative;
        });

        // reset the robot heading to 0
        driver.startButton.whenPressed(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        });

        // lock the modules in a "X" alignment
        driver.xButton.whileHeld(()->{
            SwerveModuleState[] states = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            };
            drivetrain.setModuleStates(states, false, true);
        }, drivetrain);

        controller.povDownButton.whenPressed(()-> climber.setClimberVolts(-4), climber)
        .whenReleased(()->climber.setClimberVolts(0), climber);

        controller.povUpButton.whenPressed(()-> climber.setClimberVolts(4), climber)
        .whenReleased(()->climber.setClimberVolts(0), climber);

        controller.leftStick.whenPressed(()->intake.setVoltage(-8), intake)
        .whenReleased(()->intake.setVoltage(0), intake);
        
        controller.rightTriggerButton.whenPressed(superstructure.intakeIndexBalls())
        .whenReleased(()->{
            indexer.setVoltage(0);
            intake.setVoltage(0);
        }, indexer, intake);
        controller.xButton.whenPressed(()->intake.setExtended(false));
        controller.leftBumper.whenPressed(superstructure.fenderShoot())
        .whenReleased(()->{
            shooter.setRPM(0);
            indexer.setVoltage(0);
        }, shooter, indexer);

        controller.leftTriggerButton.whenPressed(superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC())
        .whenReleased(()->{
            shooter.setRPM(0);
            indexer.setVoltage(0);
        }, drivetrain, shooter, indexer);
        
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        return sum;
    }

    public void log(){
        drivetrain.log();
        if(!DriverStation.isFMSAttached()) NetworkTableInstance.getDefault().flush();
    }
}
