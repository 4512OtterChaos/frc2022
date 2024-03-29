package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<Command> autoOptions = new SendableChooser<>();
    private PathPlannerTrajectory tripleRightTrajectory = PathPlanner.loadPath("TripleRight1", 1, 1);

    public AutoOptions(Climber climber, SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter, Superstructure superstructure){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.stop(), drivetrain)
        );
        
        autoOptions.addOption("TripleRight", 
            superstructure.autoShoot(1.5)
            .beforeStarting(()->{
                drivetrain.resetOdometry(new Pose2d(
                    tripleRightTrajectory.getInitialPose().getTranslation(), 
                    tripleRightTrajectory.getInitialState().holonomicRotation));
            })
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(superstructure.autoShoot(3))
            .andThen(superstructure.stop())
        );

        autoOptions.addOption("FenderTripleRight", 
            superstructure.fenderShootHigh(2)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRightFender1", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
                .andThen(()->drivetrain.stop(), drivetrain)
            )
            .andThen(
                superstructure.fenderShootHigh(3)
            )
            .andThen(superstructure.stop())
        );
        
        autoOptions.addOption("DoubleLeft",
            new InstantCommand(
                ()->intake.setExtended(true), 
                intake
                
                )
            .andThen(new WaitCommand(2))
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeft1", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ) 
                .deadlineWith(
                    superstructure.intakeIndexCargo(),
                    superstructure.autoHood()
                )
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(superstructure.autoShoot(3))
            .andThen(superstructure.stop())
        );

        autoOptions.addOption("DoubleLeft but troll", 
            new InstantCommand(
                ()->intake.setExtended(true), 
                intake
                
                )
            .andThen(new WaitCommand(2))    
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeft1", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ) 
                .deadlineWith(
                    superstructure.intakeIndexCargo(),
                    superstructure.autoHood()
                )
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(superstructure.autoShoot(4))
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeftTroll2", 
                    AutoConstants.kSlowSpeedConfig, 
                    false
                )
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.dumpCargo()
                .withTimeout(3)
            )

            
        );

        autoOptions.addOption("FenderDoubleLeft",
            autoFollowTrajectory(
                drivetrain, 
                "DoubleLeft1", 
                AutoConstants.kSlowSpeedConfig,
                true
            ) 
            .deadlineWith(superstructure.intakeIndexCargo())
            .andThen(autoFollowTrajectory(
                drivetrain, 
                "FenderDoubleLeft2", 
                AutoConstants.kSlowSpeedConfig,
                false
                )
                .andThen(()->drivetrain.stop(), drivetrain)
            )
            .andThen(
                superstructure.fenderShootHigh(3)
            )
            .andThen(superstructure.stop())
           
        );
        /*
        autoOptions.addOption("-1auto",
            new WaitCommand(2)
            .deadlineWith(superstructure.dumpCargo())
            
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "sabotage", 
                    AutoConstants.kMediumSpeedConfig, 
                    true
                )
            )
            
            .andThen(superstructure.stopDrive())
            
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "sabotage2", 
                    AutoConstants.kMediumSpeedConfig, 
                    false)
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            
            .andThen(
                superstructure.autoShoot(2.5)
            )
            
        );
        */
        /*
        autoOptions.addOption("QuintupleLeft", 
            autoFollowTrajectory(
                drivetrain, 
                "QuintetLeft1", 
                AutoConstants.kMediumSpeedConfig, 
                true
                )
            .deadlineWith(superstructure.intakeIndexCargo())
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.autoShoot(2.5)
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetLeft2", 
                    AutoConstants.kMediumSpeedConfig, 
                    false
                )
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.autoShoot(2.5)
            )
            .andThen(
                autoFollowTrajectory(
                drivetrain, 
                "QuintetLeft3", 
                AutoConstants.kMediumSpeedConfig, 
                false)
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.autoShoot(2.5)
            )
            
            
        );
        */
        /*
        autoOptions.addOption("FenderQuintupleRight",
            superstructure.fenderShootHigh(1.75)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRightFender1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.fenderShootHigh(1.5)
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRightFender2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                new WaitCommand(0.75)
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintupleRightFender3", 
                    AutoConstants.kFastSpeedConfig,
                    false
                ) 
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.fenderShootHigh(3)
            )
            .andThen(superstructure.stop())

        );
        */
        autoOptions.addOption("QuintupleRight",
            
            superstructure.autoShoot(2)
            .beforeStarting(()->{
                drivetrain.resetOdometry(new Pose2d(
                    tripleRightTrajectory.getInitialPose().getTranslation(), 
                    tripleRightTrajectory.getInitialState().holonomicRotation));
            })
            .alongWith(
                new InstantCommand(()-> intake.setExtended(true), 
                    intake
                )
            )
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ) 
                .deadlineWith(
                    superstructure.intakeIndexCargo(drivetrain::getLinearVelocity),
                    superstructure.autoHood()
                )
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.autoShoot(2.5)
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
                .deadlineWith(
                    superstructure.intakeIndexCargo(drivetrain::getLinearVelocity)
                )
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                new WaitCommand(0.5)
                .deadlineWith(superstructure.intakeIndexCargo(drivetrain::getLinearVelocity))
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight3", 
                    AutoConstants.kFastSpeedConfig,
                    false
                )
                .deadlineWith(
                    superstructure.autoHood()
                )
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.autoShoot(3)
            )
            .andThen(superstructure.stop())

        );

        //Don't use, no odometry; only as last resort 
        autoOptions.addOption("TaxiLastResort",
            new WaitCommand(2)
            .deadlineWith(new RunCommand(()->drivetrain.drive(0.6, 0, 0, false), drivetrain))
            .andThen(()->drivetrain.stop(), drivetrain)
        );

        autoOptions.addOption("ShootThenTaxiLastResort",
            superstructure.autoShoot(2)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                new WaitCommand(2)
                .deadlineWith(new RunCommand(()->drivetrain.drive(2, 0, 0, false), drivetrain))
            )
            .andThen(()->drivetrain.stop(), drivetrain)
        );
        
    }

    /**
     * @param trajectories Any number of trajectories to perform in sequence
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowTrajectory(SwerveDrive drivetrain, Trajectory trajectory, boolean firstTrajectory){
        final Pose2d initial = (trajectory instanceof PathPlannerTrajectory) ?
            new Pose2d(
                trajectory.getInitialPose().getTranslation(),
                ((PathPlannerState)((PathPlannerTrajectory)trajectory).sample(0)).holonomicRotation)
            :
            trajectory.getInitialPose();
        Command followCommand = new OCSwerveFollower(drivetrain, trajectory);
        if(firstTrajectory){
            followCommand = followCommand.beforeStarting(()->drivetrain.resetOdometry(initial));
        }   
        return followCommand;
    }
    /**
     * @param config The config for this trajectory defining max velocity and acceleration
     * @param storedPathNames The names of the PathPlanner paths saved to this project for use in this trajectory
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowTrajectory(SwerveDrive drivetrain, String storedPathName, TrajectoryConfig config, boolean firstTrajectory){
        Trajectory trajectory = PathPlanner.loadPath(storedPathName, config.getMaxVelocity(), config.getMaxAcceleration(), config.isReversed());
        return autoFollowTrajectory(drivetrain, trajectory, firstTrajectory);
    }

    // Network Tables
    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
