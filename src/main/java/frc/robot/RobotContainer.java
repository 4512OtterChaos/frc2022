package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.common.ShotMap;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.FieldUtil;

public class RobotContainer {
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final ShotMap shotMap = new ShotMap(shooter);
    private final Superstructure superstructure = new Superstructure(climber, drivetrain, indexer, intake, shooter, shotMap);

    private final OCXboxController driver = new OCXboxController(0);
    private boolean isFieldRelative = true;

    private final AutoOptions autoOptions = new AutoOptions(climber, drivetrain, indexer, intake, shooter, shotMap, superstructure);

    public RobotContainer(){

        configureDriverBinds(driver);
        //configureTestBinds(driver);

        autoOptions.submit();

        //LiveWindow.disableAllTelemetry();
    }

    public void periodic(){
    }

    public Command getAutoCommand(){
        return autoOptions.getSelected();
    }

    public void disable(){
        drivetrain.stop();
        shooter.stop();
    }

    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
    }

    private void configureDriverBinds(OCXboxController driver){
        // when no other command is using the drivetrain, we
        // pass the joysticks for forward, strafe, and angular control
        Command teleopDrive = new RunCommand(()->{
            drivetrain.drive(
                driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                driver.getTurn() * drivetrain.getMaxAngularVelocityRadians(),
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
        //Climber down
        driver.povDownButton.whenPressed(()-> climber.setVolts(-8), climber)
        .whenReleased(()->climber.setVolts(0), climber);
        
        //Climber up
        driver.povUpButton.whenPressed(()-> climber.setVolts(8), climber)
        .whenReleased(()->climber.setVolts(0), climber);
        //Clear intake and indexer
        driver.leftStick
            .whenPressed(()->{
                intake.setVoltage(-4);
                indexer.setVoltage(-1);
            }, intake, indexer)
            .whenReleased(()->{
                intake.setVoltage(0);
                indexer.setVoltage(0);
            }, intake, indexer);
        
        //
        driver.rightTriggerButton.whenPressed(superstructure.intakeIndexBalls())
        .whenReleased(()->{
            indexer.setVoltage(0);
            intake.setVoltage(0);
        }, indexer, intake);
        
        /*driver.rightTriggerButton
            .whenPressed(()->{
                intake.setVoltage(4);
                indexer.setVoltage(4);
            }, intake, indexer)
            .whenReleased(()->{
                intake.setVoltage(0);
                indexer.setVoltage(0);
            }, intake, indexer);
            */
        //driver.xButton.whenPressed(()->intake.setExtended(false));
        driver.leftBumper.whenPressed(superstructure.fenderShoot())
        .whenReleased(()->{
            shooter.setRPM(0);
            indexer.setVoltage(0);
        }, shooter, indexer);

        driver.aButton.whenPressed(superstructure.fenderShoot2())
        .whenReleased(()->{
            shooter.setRPM(0);
            indexer.setVoltage(0);
        }, shooter, indexer);

        driver.leftTriggerButton.whenPressed(
            superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(
                ()->driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                ()->driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                true
            )
        )
        .whenReleased(()->{
            shooter.setRPM(0);
            indexer.setVoltage(0);
        }, drivetrain, shooter, indexer);

        // estimate hood angle continuously before shooting
        shooter.setDefaultCommand(new RunCommand(()->{
            shooter.setHood(
                /*shotMap.find(
                    Units.metersToInches(
                        drivetrain.getPose().getTranslation().getDistance(FieldUtil.kFieldCenter);
                    )
                ).hoodMM
                */
                0
            );
        }, shooter));
    }
    // Manual shot tuning
    private void configureTestBinds(OCXboxController driver){
        Command teleopDrive = new RunCommand(()->{
            drivetrain.drive(
                driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                driver.getTurn() * drivetrain.getMaxAngularVelocityRadians(),
                true,
                isFieldRelative);
        }, drivetrain).beforeStarting(()->driver.resetLimiters());
        drivetrain.setDefaultCommand(teleopDrive);

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
        //Clear intake and indexer 
        driver.leftStick
            .whenPressed(()->{
                intake.setVoltage(-8);
                indexer.setVoltage(-8);
            }, intake, indexer)
            .whenReleased(()->{
                intake.setVoltage(0);
                indexer.setVoltage(0);
            }, intake, indexer);

        shooter.setDefaultCommand(new RunCommand(()->{
            double hoodDelta = 0;
            if(driver.povUpButton.get()) hoodDelta += 1;
            if(driver.povDownButton.get()) hoodDelta -= 1;
            shooter.setHood(shooter.getState().hoodMM + hoodDelta);

            shooter.setRPM(driver.getLeftTriggerAxis() * ShooterConstants.kMaxRPM);
            //shooter.setShooterVoltage(driver.getLeftTriggerAxis()*12);
        }, shooter));

        driver.rightTriggerButton
            .whenPressed(superstructure.intakeIndexBalls())
            .whenReleased(()->{
                indexer.setVoltage(0);
                intake.setVoltage(0);
            }, indexer, intake);

        driver.rightBumper
            .whenPressed(()->indexer.setVoltage(7), indexer)
            .whenReleased(()->indexer.setVoltage(0), indexer);
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        sum += shooter.getCurrentDraw();
        sum += indexer.getCurrentDraw();
        sum += intake.getCurrentDraw();
        return sum;
    }

    public void log(){
        Translation2d driveTranslation = drivetrain.getPose().getTranslation();
        SmartDashboard.putNumber("Shooter/DistanceInches", Units.metersToInches(driveTranslation.getDistance(FieldUtil.kFieldCenter)));
        drivetrain.log();
        indexer.log();
        shooter.log();
        climber.log();
        if(!DriverStation.isFMSAttached()) NetworkTableInstance.getDefault().flush();
    }
}
