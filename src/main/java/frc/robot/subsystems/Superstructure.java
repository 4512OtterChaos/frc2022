// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotMap;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

@Log.Exclude
public class Superstructure extends SubsystemBase implements Loggable {

    private final Climber climber;
    private final SwerveDrive drivetrain;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final Vision vision;

    @Log
    private int cargoStored = 0;

    public Superstructure(Climber climber, SwerveDrive drivetrain, Indexer indexer, Intake intake, Shooter shooter, Vision vision) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.vision = vision;
    }
    
    @Override
    public void periodic() {
        // adjust odometry with vision data
        // without this, pose estimation is equivalent to normal odometry
        
        Translation2d target = vision.getRobotToTargetTranslation();
        boolean visionOnEdge = Math.abs(vision.getTargetYaw().getDegrees()) > 25;
        if(vision.getHasTarget() && !visionOnEdge){
            drivetrain.addVisionMeasurement(
                new Pose2d(
                    FieldUtil.kFieldCenter.minus(target.rotateBy(drivetrain.getHeading())),
                    drivetrain.getPose(vision.getLatencySeconds()).getRotation()
                ),
                vision.getLatencySeconds()
            );
        }
        
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
        return new InstantCommand(()->intake.setExtended(true), intake)
        .andThen(new ConditionalCommand(
            new InstantCommand(), 
            new WaitCommand(0.3), 
            intake::getExtended
        ))
        .andThen(new StartEndCommand(
            ()->intake.setVoltageIn(), 
            ()->intake.stop(),
            intake
        ));
    }
    public Command intakeCargo(DoubleSupplier linearVelocity){
        return new InstantCommand(()->intake.setExtended(true), intake)
        .andThen(new ConditionalCommand(
            new InstantCommand(), 
            new WaitCommand(0.3), 
            intake::getExtended
        ))
        .andThen(new FunctionalCommand(
            ()->{},
            ()->{
                intake.setVoltage(IntakeConstants.kVoltageIn+Math.abs(linearVelocity.getAsDouble()));
            }, 
            (interrupted)->intake.stop(),
            ()->false,
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
     * Perpetually reverses intake/indexer to dump any cargo inside the robot.
     * This command must be interrupted and will stop intaking/indexing once interrupted.
     */
    public Command dumpCargo(){
        return new StartEndCommand(
            ()->{
                intake.setExtended(true);
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
     * Perpetually feeds cargo to the shooter based on the supplied condition.
     * This command must be interrupted and will stop indexing once interrupted.
     * @param condition The condition of whether to feed cargo or not
     */
    public Command feedCargo(BooleanSupplier condition){
        return new FunctionalCommand(
            ()->{},
            ()->{
                if(condition.getAsBoolean()){
                    indexer.setVoltageFeed();
                }
                else{
                    indexer.stop();
                }
            },
            (interrupted)->{indexer.stop();},
            ()->false, indexer
        );
    }
    /**
     * Perpetually feeds cargo to the shooter.
     * This command must be interrupted and will stop indexing once interrupted.
     */
    public Command feedCargo(){
        return feedCargo(()->true);
    }

    /**
     * Perpetually intakes and indexes cargo automatically.
     * This command must be interrupted and will stop intaking/indexing once interrupted.
     */
    public Command intakeIndexCargo(){
        return intakeCargo()
            .alongWith(indexCargo());
    }

    public Command intakeIndexCargo(DoubleSupplier linearVelocity){
        return intakeCargo(linearVelocity)
            .alongWith(indexCargo());
    }

    /**
     * Counts cargo amount internally and optionally rumbles controllers.
     * (Ends immediately, schedules rumble commands)
     * @param amount Amount of cargo to count (e.g. +1 cargo intaked)
     * @param controllers Controllers to optionally rumble on positive cargo counted
     */
    public Command countCargo(int amount, OCXboxController... controllers) {
        return new InstantCommand(()->{
            int amnt = amount;
            boolean intaking = false;
            for(OCXboxController controller : controllers) {
                intaking = intaking || controller.rightTriggerButton.get();
            }
            if(controllers.length > 0 && !intaking && !DriverStation.isAutonomous()) {
                amnt = Math.min(amnt, 0);
            }

            // count
            cargoStored = MathUtil.clamp(cargoStored + amnt, 0, 2);

            // +1 cargo rumble
            if(amnt > 0) {
                new RunCommand(()->{
                    for(OCXboxController controller : controllers) {
                        controller.rumble(true, 1);
                    }
                }).withTimeout(0.4)
                .andThen(()->{
                    for(OCXboxController controller : controllers) {
                        controller.rumble(true, 0);
                    }
                }).schedule();
            }
            // Empty cargo rumble
            if(amnt < 0 && cargoStored == 0) {
                new RunCommand(()->{
                    for(OCXboxController controller : controllers) {
                        controller.rumble(false, 1);
                    }
                }).withTimeout(0.2)
                .andThen(()->{
                    for(OCXboxController controller : controllers) {
                        controller.rumble(false, 0);
                    }
                }).schedule();
            }
        });
    }
    /**
     * Pulse-rumbles controllers while cargo is full (RunCommand)
     * @param controllers Controllers to rumble on cargo full
     */
    public Command rumbleCargoFull(OCXboxController... controllers) {
        return new FunctionalCommand(
            ()->{},
            ()->{
                if(cargoStored == 2) {
                    double time = Timer.getFPGATimestamp() / 0.1; // time / x seconds per pulse
                    boolean pulse = ((int) time) % 2 == 0;
                    for(OCXboxController controller : controllers) {
                        if(pulse) controller.rumble(false, 1);
                        else controller.rumble(false, 0);
                    }
                }
                else {
                    for(OCXboxController controller : controllers) {
                        controller.rumble(false, 0);
                    }
                }
            },
            (interrupted)->{
                for(OCXboxController controller : controllers) {
                    controller.rumble(false, 0);
                } 
            },
            ()->false
        );
    }
    /**
     * Ends when stored cargo is 0.
     * @param controllers Controllers to rumble on cargo empty
     */
    public Command checkCargoEmpty(OCXboxController... controllers) {
        return new InstantCommand().perpetually()
        .withInterrupt(()->cargoStored <= 0);
    }

    /**
     * Set the target shooter state perpetually.
     * Stops the shooter when interrupted.
     */
    public Command setShooterState(Shooter.State state){
        return new StartEndCommand(
            ()->shooter.setState(state),
            ()->shooter.stop(),
            shooter
        );
    }
    /**
     * Set the target shooter state and finishes when the shooter is within tolerance or
     * timeout seconds have passed.
     * Does NOT stop the shooter when the state is achieved.
     */
    public Command setShooterState(Shooter.State state, double timeout){
        return new InstantCommand(()->shooter.setState(state), shooter)
        .perpetually()
        .withInterrupt(()->shooter.getState().withinTolerance(state))
        .withTimeout(timeout);
    }

    /**
     * Shoot in the high goal from against the fender perpetually.
     * This command must be interrupted and will stop shooting/indexing once interrupted.
     */
    public Command fenderShootHigh(){
        return setShooterState(ShotMap.find(0)) // closest state (fender)
        .alongWith(feedCargo(shooter::withinTolerance));
    }
    /**
     * Shoot in the high goal from against the fender for timeout seconds
     * and then stop the indexer/shooter.
     */
    public Command fenderShootHigh(double timeout){
        return new WaitCommand(timeout)
        .deadlineWith(fenderShootHigh());
    }
    /**
     * Shoot in the low goal from against the fender perpetually.
     * This command must be interrupted and will stop shooting/indexing once interrupted.
     */
    public Command fenderShootLow(){
        return setShooterState(ShotMap.kFenderLow)
        .alongWith(feedCargo(shooter::withinTolerance));
    }
    /**
     * Shoot in the low goal from against the fender for timeout seconds
     * and then stop the indexer/shooter.
     */
    public Command fenderShootLow(double timeout){
        return new WaitCommand(timeout)
        .deadlineWith(fenderShootLow());
    }
    
    /**
     * Automatically aims and fires at the high goal (odometry must be correct!).
     * The velocity suppliers allow translation while aiming/firing.
     * This command is perpetual and will stop the drivebase/indexer/shooter
     * when interrupted.
     * @param vxMeters X velocity supplier
     * @param vyMeters Y velocity supplier
     * @param openLoop If should not use velocity PID on swerve modules
     * @param targetTranslation The translation of the target to aim at on the field
     * @param targetingCondition Any additional consideration that determines when
     * the target angle should be updated and also when the indexer is ready to feed(in addition
     * to shooter/drive tolerance).
     */
    public Command autoShoot(
            DoubleSupplier vxMeters, DoubleSupplier vyMeters, boolean openLoop,
            Translation2d targetTranslation,
            BooleanSupplier targetingCondition
        ){
        return new FunctionalCommand(
            ()->{
                drivetrain.resetPathController();
            }, 
            ()->{
                boolean targetingReady = targetingCondition.getAsBoolean();
                //shooter setState to odometry distance
                Translation2d driveTranslation = drivetrain.getPose().getTranslation();
                Translation2d translationToTarget = targetTranslation.minus(driveTranslation);
                double distance = translationToTarget.getNorm();

                Shooter.State targetShooterState = ShotMap.find(distance);
                shooter.setState(targetShooterState);
                //Drivetrain heading target to hub
                Rotation2d angleToTarget = drivetrain.getHeading();
                if(targetingReady){
                    angleToTarget = new Rotation2d(translationToTarget.getX(), translationToTarget.getY())
                        .plus(new Rotation2d(Math.PI));
                }
                boolean driveAtGoal = drivetrain.drive(
                    vxMeters.getAsDouble(),
                    vyMeters.getAsDouble(),
                    angleToTarget,
                    openLoop
                );
                //Indexer feed when shooter && drivetrain ready
                if(driveAtGoal && shooter.withinTolerance() && targetingReady){
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
     * Automatically aims and fires at the high goal (odometry must be correct!).
     * The velocity suppliers allow translation while aiming/firing.
     * This command is perpetual and will stop the drivebase/indexer/shooter
     * when interrupted.
     * @param vxMeters X velocity supplier
     * @param vyMeters Y velocity supplier
     * @param openLoop If should not use velocity PID on swerve modules
     * @param readyCondition Any boolean condition in addition to drivetrain angle
     * and shooter state tolerance to decide when to begin feeding the indexer
     */
    public Command autoShoot(
            DoubleSupplier vxMeters, DoubleSupplier vyMeters, boolean openLoop
        ){
        return autoShoot(vxMeters, vyMeters, openLoop, FieldUtil.kFieldCenter, ()->true);
    }
    /**
     * Automatically aims in place and fires at the high goal (odometry must be correct!).
     * This overload is intended for autonomous fire-in-place and uses closed-loop control.
     * This command will end after timeout seconds and stop the drivebase/indexer/shooter.
     */
    public Command autoShoot(double timeout){
        return autoShoot(()->0, ()->0, false)
            .withTimeout(timeout);
    }

    /**
     * Automatically adjust the hood angle based on current odom distance to hub.
     * (This is the shooter default command)
     */
    public Command autoHood() {
        return new RunCommand(()->{
            shooter.setHood(
                ShotMap.find(
                    drivetrain.getPose().getTranslation().getDistance(FieldUtil.kFieldCenter)
                    
                ).hoodMM
                
                //0
            );
        }, shooter);
    }
}
