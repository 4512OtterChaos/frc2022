package frc.robot.auto;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.OCSwerveFollower;
import frc.robot.common.ShotMap;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(Climber climber, Drivetrain drivetrain, Indexer indexer, Intake intake, Shooter shooter, ShotMap shotMap, Superstructure superstructure){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.stop(), drivetrain)
        );
        autoOptions.addOption("TripleRight", 
            superstructure.fenderShoot()
            .withTimeout(3)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "TripleRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexBalls())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3))
        );
        
        autoOptions.addOption("DoubleLeft", 
            superstructure.fenderShoot()
            .withTimeout(3)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "DoubleLeft1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexBalls())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3))
            

        );
        autoOptions.addOption("QuintupleRight", 
            superstructure.fenderShoot()
            .withTimeout(3)
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
                .deadlineWith(superstructure.intakeIndexBalls())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                superstructure.otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(3)
            )
            .andThen(
                autoFollowTrajectory(
                    drivetrain, 
                    "QuintetRight2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
                .deadlineWith(superstructure.intakeIndexBalls())
            )
            .andThen(()->drivetrain.stop(), drivetrain)
            .andThen(
                new WaitCommand(2)
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

        );

        //Don't use, no odometry; only as last resort 
        autoOptions.addOption("TaxiLastResort", 
            new RunCommand(()->drivetrain.drive(0.4, 0, 0, false, false), drivetrain)
            .withTimeout(3)
            .andThen(()->drivetrain.stop(), drivetrain)
        );

        autoOptions.addOption("ShootThenTaxiLastResort", 
            superstructure.fenderShoot()
            .withTimeout(3)
            .andThen(
                new RunCommand(()->drivetrain.drive(0.4, 0, 0, false, false), drivetrain)
                .withTimeout(3)
            )
            .andThen(()->drivetrain.stop(), drivetrain)
        );
    }

    /**
     * @param trajectories Any number of trajectories to perform in sequence
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowTrajectory(Drivetrain drivetrain, Trajectory trajectory, boolean firstTrajectory){
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
    private Command autoFollowTrajectory(Drivetrain drivetrain, String storedPathName, TrajectoryConfig config, boolean firstTrajectory){
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
