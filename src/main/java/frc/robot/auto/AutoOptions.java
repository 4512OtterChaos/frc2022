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

    public AutoOptions(Climber climber, SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter, Superstructure superstructure){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.stop(), drivetrain)
        );
        
        autoOptions.addOption("TripleRight", 
            superstructure.fenderShootHigh(2.5)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3))
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
                    AutoConstants.kMediumSpeedConfig,
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
            superstructure.fenderShootHigh(2.5)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeft1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
                .andThen(()->drivetrain.stop(), drivetrain)
            )
            .andThen(superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3))
            .andThen(superstructure.stop())
        );
        autoOptions.addOption("FenderDoubleLeft",
            superstructure.fenderShootHigh(2.5)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeft1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(autoFollowTrajectory(
                drivetrain, 
                "DoubleLeft2", 
                AutoConstants.kMediumSpeedConfig,
                false
                )
                .andThen(()->drivetrain.stop(), drivetrain)
            )
            .andThen(
                superstructure.fenderShootHigh(3)
            )
            .andThen(superstructure.stop())
           
        );
        autoOptions.addOption("QuintupleRight",
            superstructure.fenderShootHigh(1.5)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexCargo())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(2.5)
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight2", 
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
                    "QuintetRight3", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ) 
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3)
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
            superstructure.fenderShootHigh(2)
            .andThen(()->{
                shooter.stop();
                indexer.stop();
            }, indexer, shooter)
            .andThen(
                new WaitCommand(2)
                .deadlineWith(new RunCommand(()->drivetrain.drive(0.6, 0, 0, false), drivetrain))
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
