package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.simulation.CargoSim;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TeleopDriveAngle;
import frc.robot.subsystems.drivetrain.commands.TeleopDriveBasic;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotMap;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtil;

public class RobotContainer {
    private final Climber climber = new Climber();
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Vision vision = new Vision();
    private final Superstructure superstructure = new Superstructure(climber, drivetrain, indexer, intake, shooter, vision);

    private final OCXboxController driver = new OCXboxController(0);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final AutoOptions autoOptions = new AutoOptions(climber, drivetrain, indexer, intake, shooter, superstructure);

    private final Field2d field = new Field2d();

    public RobotContainer(){

        configureDriverBinds(driver);
        //configureTestBinds(driver);

        autoOptions.submit();
        SmartDashboard.putData("Field", field);

        LiveWindow.disableAllTelemetry();
        SmartDashboard.putNumber("Shooter/RPM Offset", 0);
    }

    public void periodic(){
        
        ShotMap.setRPMOffset(SmartDashboard.getNumber("Shooter/RPM Offset", 0));
    }

    public Command getAutoCommand(){
        return autoOptions.getSelected();
    }

    public void disable(){
        drivetrain.stop();
        intake.stop();
        indexer.stop();
        shooter.stop();
        climber.stop();
        shooter.setHood(0);
    }

    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
        intake.setBrakeOn(is);
        indexer.setBrakeOn(is);
    }

    private void configureDriverBinds(OCXboxController controller){
        // when no other command is using the drivetrain, we
        // pass the joysticks for forward, strafe, and angular position control
        drivetrain.setDefaultCommand(new TeleopDriveBasic(controller, drivetrain));
        //drivetrain.setDefaultCommand(new TeleopDriveAngle(controller, drivetrain));
        // push-to-change driving "speed"
        controller.rightBumper
            .whenPressed(()->controller.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault));

        // toggle between field-relative and robot-relative control
        controller.backButton.whenPressed(()->{
            drivetrain.setIsFieldRelative(!drivetrain.getIsFieldRelative());
        });

        // reset the robot heading to 0
        controller.startButton.whenPressed(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        });

        // lock the modules in a "X" alignment
        controller.xButton.whileHeld(()->{
            SwerveModuleState[] states = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            };
            drivetrain.setModuleStates(states, false, true);
        }, drivetrain);

        //Climber down
        controller.povDownButton
            .whenPressed(()->climber.setVolts(-10), climber)
            .whenReleased(()->climber.setVolts(0), climber);
        //Climber up
        controller.povUpButton
            .whenPressed(()->climber.setVolts(10), climber)
            .whenReleased(()->climber.setVolts(0), climber);

        //Clear intake and indexer
        controller.leftStick
            .whenPressed(()->{
                intake.setVoltageOut();
                indexer.setVoltageOut();
            }, intake, indexer)
            .whenReleased(()->{
                intake.stop();
                indexer.stop();
            }, intake, indexer);
        
        // intake and automatically index cargo, rumble based on status
        controller.rightTriggerButton
            .whenPressed(
                superstructure.intakeIndexCargo()
                .deadlineWith(
                    new RunCommand(()->{
                        if(indexer.getTopSensed()){
                            // pulsing rumble when full
                            double rumble = Math.sin(Timer.getFPGATimestamp()*20);
                            rumble = rumble < 0 ? 0 : 0.5;
                            controller.rumble(rumble);
                        }
                        else if(indexer.shouldIndex()) controller.rumble(0.25);
                        else controller.rumble(0);
                    })
                )
            )
            .whenReleased(
                superstructure.stopIntake()
                .alongWith(
                    superstructure.stopIndexer(),
                    new InstantCommand(()->controller.rumble(0))
                )
            );
        
        controller.bButton.whenPressed(()->intake.setExtended(false));

        controller.yButton.whenPressed(superstructure.fenderShootHigh())
        .whenReleased(()->{
            shooter.stop();
            indexer.stop();
        }, shooter, indexer);

        controller.aButton.whenPressed(superstructure.fenderShootLow())
        .whenReleased(()->{
            shooter.stop();
            indexer.stop();
        }, shooter, indexer);

        // auto shoot high hub with pose estimation
        controller.leftTriggerButton.whenPressed(
            superstructure.autoShoot(
                ()->driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                ()->driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                false
            ).beforeStarting(()->controller.resetLimiters())
        )
        .whenReleased(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );

        // auto shoot high hub with only vision data
        /*
        controller.leftBumper.whenPressed(
            superstructure.autoShoot(
                ()->driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                ()->driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                true,
                //drivetrain.getPose().transformBy(vision.getRobotToTarget(drivetrain.getHeading())).getTranslation(),
                drivetrain.getPose().plus(
                    new Transform2d(vision.getRobotToTargetTranslation(), new Rotation2d())
                ).getTranslation(),
                ()->vision.getHasTarget()
            ).beforeStarting(()->controller.resetLimiters())
        )
        .whenReleased(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );
        */

        controller.leftBumper.whenPressed(
            new InstantCommand(controller::resetLimiters)
            .andThen(new FunctionalCommand(
                ()->{
                    drivetrain.resetPathController();
                    vision.resetFilter();
                }, 
                ()->{
                    Translation2d target = vision.getFilteredRobotToTargetTranslation().plus(
                        drivetrain.getPose().minus(drivetrain.getPose(vision.getLatencySeconds())).getTranslation()
                    );
                    boolean hasTarget = vision.getHasTarget();
                    double dist = target.getNorm();
                    Rotation2d targetAngle = new Rotation2d(target.getX(), target.getY())
                        .plus(new Rotation2d(Math.PI))
                        .plus(drivetrain.getHeading());
                    Shooter.State targetShooterState = ShotMap.find(dist);
                    shooter.setState(targetShooterState);
                    //Drivetrain heading target to hub
                    boolean driveAtGoal = drivetrain.drive(
                        driver.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                        driver.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                        targetAngle,
                        true
                    );
                    //Indexer feed when shooter && drivetrain ready
                    if(driveAtGoal && shooter.withinTolerance() && hasTarget){
                        indexer.setVoltageFeed();
                    }
                    else{
                        indexer.stop();
                    }
                }, 
                (interrupted)->{
                    shooter.stop();
                    indexer.stop();
                    drivetrain.stop();
                },
                ()->false,
                drivetrain, shooter, indexer
            ))
        )
        .whenReleased(
            superstructure.stopDrive()
            .alongWith(
                superstructure.stopIndexer(),
                superstructure.stopShooter()
            )
        );

        // estimate hood angle continuously before shooting
        shooter.setDefaultCommand(new RunCommand(()->{
            shooter.setHood(
                ShotMap.find(
                    drivetrain.getPose().getTranslation().getDistance(FieldUtil.kFieldCenter)
                    
                ).hoodMM
                
                //0
            );
        }, shooter));
    }
    // Manual shot tuning
    private void configureTestBinds(OCXboxController controller){
        drivetrain.setDefaultCommand(new TeleopDriveBasic(controller, drivetrain));

        // toggle between field-relative and robot-relative control
        controller.backButton.whenPressed(()->{
            drivetrain.setIsFieldRelative(!drivetrain.getIsFieldRelative());
        });

        // reset the robot heading to 0
        controller.startButton.whenPressed(()->{
            drivetrain.resetOdometry(
                new Pose2d(
                    drivetrain.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        });
        //Clear intake and indexer
        controller.leftStick
            .whenPressed(()->{
                intake.setVoltageOut();
                indexer.setVoltageOut();
            }, intake, indexer)
            .whenReleased(()->{
                intake.stop();
                indexer.stop();
            }, intake, indexer);

        shooter.setDefaultCommand(new RunCommand(()->{
            shooter.setHood(SmartDashboard.getNumber("Hood MM", 0));
            shooter.setRPM(SmartDashboard.getNumber("Shooter Rpm", 0));

            //shooter.setShooterVoltage(controller.getLeftTriggerAxis()*12);
        }, shooter));

        // intake and automatically index cargo, rumble based on status
        controller.rightTriggerButton
            .whenPressed(
                superstructure.intakeIndexCargo()
                .deadlineWith(
                    new RunCommand(()->{
                        if(indexer.getTopSensed()){
                            // pulsing rumble when full
                            double rumble = Math.sin(Timer.getFPGATimestamp()*20);
                            rumble = rumble < 0 ? 0 : 0.5;
                            controller.rumble(rumble);
                        }
                        else if(indexer.shouldIndex()) controller.rumble(0.25);
                        else controller.rumble(0);
                    })
                )
            )
            .whenReleased(
                superstructure.stopIntake()
                .alongWith(
                    superstructure.stopIndexer(),
                    new InstantCommand(()->controller.rumble(0))
                )
            );

        controller.leftTriggerButton
            .whenPressed(()->indexer.setVoltageFeed(), indexer)
            .whenReleased(()->indexer.stop(), indexer);
    }

    public void log(){
        drivetrain.log();
        intake.log();
        indexer.log();
        shooter.log();
        climber.log();
        vision.log();
        
        SmartDashboard.putBoolean("Comp/Switch", compressor.getPressureSwitchValue());

        field.setRobotPose(drivetrain.getPose());
        field.getObject("vision pose").setPose(new Pose2d(
            FieldUtil.kFieldCenter.minus(vision.getRobotToTargetTranslation().rotateBy(drivetrain.getHeading())),
            new Rotation2d()
        ));
        //field.getObject("Vision Target").setPose(drivetrain.getPose().transformBy(vision.getRobotToTarget(drivetrain.getHeading())));
        field.getObject("Vision Target").setPose(drivetrain.getPose().plus(
            new Transform2d(vision.getRobotToTargetTranslation(), new Rotation2d())
        ));
        
        field.getObject("Swerve Modules").setPoses(drivetrain.getModulePoses());
        Trajectory logTrajectory = drivetrain.getLogTrajectory();
        if(logTrajectory == null) logTrajectory = new Trajectory();
        field.getObject("Trajectory").setTrajectory(logTrajectory);

        Translation2d driveTranslation = drivetrain.getPose().getTranslation();
        SmartDashboard.putNumber(
            "Shooter/DistanceInches",
            Units.metersToInches(driveTranslation.getDistance(FieldUtil.kFieldCenter))
        );

        if(!DriverStation.isFMSAttached()) NetworkTableInstance.getDefault().flush();
    }



    //----- Simulation

    private final Field2d xzField = new Field2d();
    private CargoSim cargoSimulation = new CargoSim(
        drivetrain::getPose,
        drivetrain::getChassisSpeeds,
        intake::getRPM,
        indexer::getRPM,
        shooter::getState,
        indexer::setBottomSimSensed,
        indexer::setTopSimSensed,
        field,
        xzField
    );
    public void simulationInit(){
        SmartDashboard.putData("Field XZ", xzField);
    }
    public void simulationPeriodic(){
        cargoSimulation.update();
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        sum += shooter.getCurrentDraw();
        sum += indexer.getCurrentDraw();
        sum += intake.getCurrentDraw();
        return sum;
    }
}
