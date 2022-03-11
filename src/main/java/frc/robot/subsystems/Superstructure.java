// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    /** Creates a new Superstructure. */
    private final Climber climber;
    private final SwerveDrive drivetrain;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final ShotMap shotMap;
    public Superstructure(Climber climber, SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter, ShotMap shotMap) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.shotMap = shotMap;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public Command intakeIndexBalls(){
        return intakeIndexBalls((bool)->{});
    }

    public Command intakeIndexBalls(BooleanConsumer indexingCargo){
        //return new InstantCommand(()->intake.setExtended(true), intake)
        //.andThen(new WaitCommand(0.3))
        return new InstantCommand()
        .andThen(new StartEndCommand(
            ()->intake.setVoltage(4), 
            ()->intake.setVoltage(0),
            intake
        ))
        .alongWith(new FunctionalCommand(
            ()->{},
            ()->{
                if(indexer.getBottomSensed() && !indexer.getTopSensed()){
                    indexer.setVoltage(2.5);
                    indexingCargo.accept(true);
                }
                else{
                    indexer.setVoltage(0);
                    indexingCargo.accept(false);
                }
            },
            (interrupted)->indexer.setVoltage(0),
            ()->false,
            indexer));
    }
    public Command fenderShoot(){
        
        return setShooterState(shotMap.find(0))
        .andThen(()->indexer.setVoltage(8), indexer); 
    }
    public Command fenderShoot2(){
        return setShooterState(shotMap.find(130));
        //.andThen(()->indexer.setVoltage(8), indexer); 
    }
    
    //Cool shooting
    public Command otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(
        DoubleSupplier vxMeters, DoubleSupplier vyMeters, boolean fieldRelative
        ){
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                //shooter setState to odometry distance
                Translation2d driveTranslation = drivetrain.getPose().getTranslation();
                double distance = driveTranslation.getDistance(FieldUtil.kFieldCenter);

                distance = Units.metersToInches(distance);
                Shooter.State targetShooterState = shotMap.find(distance);
                shooter.setState(targetShooterState);
                //Drivetrain heading target to hub
                Rotation2d angleToCenter = FieldUtil.getAngleToCenter(driveTranslation);
                boolean driveAtGoal = drivetrain.driveRotate(
                    angleToCenter.plus(new Rotation2d(Math.PI)),
                    vxMeters.getAsDouble(),
                    vyMeters.getAsDouble(),
                    fieldRelative
                );
                //Indexer feed when shooter && drivetrain ready
                if(driveAtGoal && shooter.getState().withinTolerance(targetShooterState)){
                    indexer.setVoltage(5);
                }
                else{
                    indexer.setVoltage(0);
                }
            }, 
            (interrupted)->{
                shooter.setRPM(0);
                indexer.setVoltage(0);
            },
            ()->false,
            drivetrain, shooter, indexer
        );
    }
    public Command otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(double timeout){
        return otterChaosShootsEpicShotMOMENTWEDONTHAVEAMENAKSKNJC(()->0, ()->0, false)
            .withTimeout(timeout);
    }
    public Command setShooterState(Shooter.State state){
        return new InstantCommand(()->shooter.setState(state), shooter)
        .perpetually()
        .withInterrupt(()->shooter.getState().withinTolerance(state)
        );
    }
    
}
