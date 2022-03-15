// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotMap;
import frc.robot.util.FieldUtil;

public class Superstructure extends SubsystemBase {

    private final Climber climber;
    private final SwerveDrive drivetrain;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;

    public Superstructure(Climber climber, SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
    }
    
    @Override
    public void periodic() {
    }

    public Command stopDrive(){
        return new InstantCommand(drivetrain::stop, drivetrain);
    }
    public Command stopIntake(){
        return new InstantCommand(intake::stop, intake);
    }
    public Command stopIndexer(){
        return new InstantCommand(indexer::stop, indexer);
    }
    public Command stopShooter(){
        return new InstantCommand(shooter::stop, shooter);
    }
    public Command stopClimber(){
        return new InstantCommand(climber::stop, climber);
    }
    public Command stop(){
        return stopDrive()
            .alongWith(
                stopIntake(),
                stopIndexer(),
                stopShooter(),
                stopClimber()
            );
    }

    /**
     * Deploys and runs intake perpetually.
     * This command must be interrupted and will stop intaking once interrupted.
     */
    public Command intakeCargo(){
        //return new InstantCommand(()->intake.setExtended(true), intake)
        //.andThen(new WaitCommand(0.3))
        return new InstantCommand()
        .andThen(new StartEndCommand(
            ()->intake.setVoltage(4), 
            ()->intake.stop(),
            intake
        ));
    }

    /**
     * Perpetually reads indexer sensors and automatically indexes incoming cargo.
     * This command must be interrupted and will stop indexing once interrupted.
     */
    public Command indexCargo(){
        return new FunctionalCommand(
            ()->{},
            ()->{
                if(indexer.shouldIndex()){
                    indexer.setVoltageIn();
                }
                else{
                    indexer.stop();
                }
            },
            (interrupted)->indexer.stop(),
            ()->false,
            indexer
        );
    }

    /**
     * Perpetually intakes and indexes cargo automatically.
     * This command must be interrupted and will stop intaking/indexing once interrupted.
     */
    public Command intakeIndexCargo(){
        return intakeCargo()
            .alongWith(indexCargo());
    }

    /**
     * Perpetually reverses intake/indexer to dump any cargo inside the robot.
     * This command must be interrupted and will stop intaking/indexing once interrupted.
     */
    public Command dumpCargo(){
        return new StartEndCommand(
            ()->{
                intake.setVoltageOut();
                indexer.setVoltageOut();
            },
            ()->{
                intake.stop();
                indexer.stop();
            }, intake, indexer
        );
    }

    /**
     * Set the target shooter state and finish when the shooter is within tolerance.
     * Does NOT stop the shooter when the state is achieved.
     */
    public Command setShooterState(Shooter.State state){
        return new InstantCommand(()->shooter.setState(state), shooter)
        .perpetually()
        .withInterrupt(()->shooter.getState().withinTolerance(state))
        .withTimeout(2.5);
    }

    /**
     * Shoot in the high goal from against the fender.
     * Does NOT stop the shooter/indexer when finished, the interrupting
     * command must do this.
     */
    public Command fenderShootHigh(){
        return setShooterState(ShotMap.find(0)) // closest state
        .andThen(()->indexer.setVoltageFeed(), indexer);
    }
    /**
     * Shoot in the high goal from against the fender for timeout seconds
     * and then stop the indexer/shooter. This command will not stop the 
     * indexer/shooter if interrupted.
     */
    public Command fenderShootHigh(double timeout){
        return new WaitCommand(timeout)
            .deadlineWith(fenderShootHigh())
            .andThen(
                stopShooter(),
                stopIndexer()
            );
    }
    /**
     * Shoot in the low goal from against the fender.
     * Does NOT stop the shooter/indexer when finished, the interrupting
     * command must do this.
     */
    public Command fenderShootLow(){
        return setShooterState(ShotMap.kFenderLow)
            .andThen(()->indexer.setVoltageFeed(), indexer); 
    }
    /**
     * Shoot in the low goal from against the fender for timeout seconds
     * and then stop the indexer/shooter. This command will not stop the 
     * indexer/shooter if interrupted.
     */
    public Command fenderShootLow(double timeout){
        return new WaitCommand(timeout)
            .deadlineWith(fenderShootLow())
            .andThen(
                stopShooter(),
                stopIndexer()
            );
    }
    
    /**
     * Automatically aims and fires at the high goal (odometry must be correct!).
     * The velocity suppliers allow translation while aiming/firing.
     * This command is perpetual and will stop the drivebase/indexer/shooter
     * when interrupted.
     * @param vxMeters X velocity supplier
     * @param vyMeters Y velocity supplier
     * @param openLoop If should not use velocity PID on swerve modules
     */
    public Command otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(
        DoubleSupplier vxMeters, DoubleSupplier vyMeters, boolean openLoop
        ){
        return new FunctionalCommand(
            ()->{
                drivetrain.resetPathController();
            }, 
            ()->{
                //shooter setState to odometry distance
                Translation2d driveTranslation = drivetrain.getPose().getTranslation();
                double distance = driveTranslation.getDistance(FieldUtil.kFieldCenter);

                Shooter.State targetShooterState = ShotMap.find(distance);
                shooter.setState(targetShooterState);
                //Drivetrain heading target to hub
                Rotation2d angleToCenter = FieldUtil.getAngleToCenter(driveTranslation);
                boolean driveAtGoal = drivetrain.drive(
                    vxMeters.getAsDouble(),
                    vyMeters.getAsDouble(),
                    angleToCenter.plus(new Rotation2d(Math.PI)),
                    openLoop
                );
                //Indexer feed when shooter && drivetrain ready
                if(driveAtGoal && shooter.getState().withinTolerance(targetShooterState)){
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
        );
    }
    /**
     * Automatically aims in place and fires at the high goal (odometry must be correct!).
     * This overload is intended for autonomous fire-in-place and uses closed-loop control.
     * This command will end after timeout seconds and stop the drivebase/indexer/shooter.
     */
    public Command otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(double timeout){
        return otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(()->0, ()->0, false)
            .withTimeout(timeout);
    }
}
