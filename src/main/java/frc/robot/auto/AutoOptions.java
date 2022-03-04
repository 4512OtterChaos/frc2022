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
import frc.robot.commands.OCSwerveFollower;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(Drivetrain drivetrain){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.drive(0, 0, 0, false, false), drivetrain)
        );

        autoOptions.addOption("Ellipse",
            autoFollowInitTrajectory(
                drivetrain,
                TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                        new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
                        new Pose2d(4, 2, Rotation2d.fromDegrees(-90)),
                        new Pose2d(2, 2, Rotation2d.fromDegrees(90))
                    ),
                    AutoConstants.kMediumSpeedConfig
                )
            )
        );

        autoOptions.addOption("Test Normal",
            autoFollowInitTrajectory(
                drivetrain,
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3, 3, Rotation2d.fromDegrees(0)),
                    Arrays.asList(
                        new Translation2d(4, 3),
                        new Translation2d(4, 4),
                        new Translation2d(3, 4),
                        new Translation2d(1, 2)
                    ),
                    new Pose2d(3, 3, Rotation2d.fromDegrees(90)),
                    AutoConstants.kMediumSpeedConfig
                )
            )
        );

        // 2022 auto mockup
        autoOptions.addOption("Mockup PathPlanner",
            autoFollowInitTrajectory(
                drivetrain,
                "5 ball mockup",
                AutoConstants.kMediumSpeedConfig
            )
        );
    }

    /**
     * @param trajectories Any number of trajectories to perform in sequence
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowInitTrajectory(Drivetrain drivetrain, Trajectory trajectory){
        final Pose2d initial = (trajectory instanceof PathPlannerTrajectory) ?
            new Pose2d(
                trajectory.getInitialPose().getTranslation(),
                ((PathPlannerState)((PathPlannerTrajectory)trajectory).sample(0)).holonomicRotation)
            :
            trajectory.getInitialPose();
            
        return new OCSwerveFollower(drivetrain, trajectory).beforeStarting(()->drivetrain.resetOdometry(initial));
    }
    /**
     * @param config The config for this trajectory defining max velocity and acceleration
     * @param storedPathNames The names of the PathPlanner paths saved to this project for use in this trajectory
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowInitTrajectory(Drivetrain drivetrain, String storedPathName, TrajectoryConfig config){
        Trajectory trajectory = PathPlanner.loadPath(storedPathName, config.getMaxVelocity(), config.getMaxAcceleration(), config.isReversed());
        return autoFollowInitTrajectory(drivetrain, trajectory);
    }

    // Network Tables
    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
