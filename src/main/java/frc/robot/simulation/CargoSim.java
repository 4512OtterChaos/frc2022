package frc.robot.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldUtil;

import static frc.robot.simulation.CargoSimPhysics.*;

public class CargoSim {
    
    private static class SimCargo {
        private static int totalCargo = 0;

        private final int cargoNum;

        // distinguish idle, intaking, and shooting
        enum State{
            IDLE,
            CONTROLLED,
            FLIGHT
        }
        private State state = State.IDLE;

        public final Translation3d resetPos;
        private Translation3d pos;

        // controlled variables
        private double currentPathDistance = 0;
        // indexer sensors
        private boolean tripsBot = false;
        private boolean tripsTop = false;

        // flight variables
        private Translation3d flightVelocities = new Translation3d();
        private Rotation2d spinDirection = new Rotation2d();
        private double spinVel = 0;
        private double groundSeconds = 0;

        public SimCargo(Translation2d resetPos){
            this.resetPos = new Translation3d(resetPos.getX(), resetPos.getY(), kRadius);
            pos = this.resetPos;

            totalCargo++;
            this.cargoNum = totalCargo;
        }

        public void setPos(Translation3d pos){this.pos = pos;}
        public void setState(State state){this.state = state;}
        
        public void reset(){
            pos = resetPos;
            state = State.IDLE;
            currentPathDistance = 0;
            groundSeconds = 0;
            spinVel = 0;
            spinDirection = new Rotation2d();
            tripsBot = false;
            tripsTop = false;
            flightVelocities = new Translation3d();
        }

        public State getState(){return state;}
        public Translation3d getPos(){return pos;}

        public void update(
                double dt,
                Pose2d robotPose, ChassisSpeeds robotSpeeds,
                double intakeRPM, double indexerRPM,
                Shooter.State shooterState
            ){
            Translation3d intakePos = new Translation3d(0, 0, kIntakeHeight).plusXY(
                robotPose.transformBy(new Transform2d(
                    new Translation2d(kIntakeOffset, 0), new Rotation2d()
                )).getTranslation()
            );
            Translation3d shooterPos = new Translation3d(0, 0, kShooterHeight).plusXY(robotPose.getTranslation());

            double intakeVelocity = intakeRPM / 60 * Units.inchesToMeters(4) * Math.PI * 0.5;
            double indexVelocity = indexerRPM / 60 * Units.inchesToMeters(4) * Math.PI * 0.05;

            Rotation2d robotYaw = robotPose.getRotation();

            double shotVelocity = shooterState.rpm / 60 * Units.inchesToMeters(4) * Math.PI * 0.6;
            double shotSpinVelocity = shooterState.rpm / 60 * Units.inchesToMeters(4) * Math.PI * 0.025;
            Rotation2d shotPitch = Rotation2d.fromDegrees(
                MathUtil.interpolate(85, 40, shooterState.hoodMM/ShooterConstants.kServoLengthMM)
            );
            Rotation2d shotYaw = robotYaw.plus(new Rotation2d(Math.PI));

            if(state == State.FLIGHT){
                // z velocity after gravity
                flightVelocities = flightVelocities.plusZ(
                    kGravityAccel * dt
                );
                // air drag
                double currentVelocity = flightVelocities.getDistance();
                flightVelocities = flightVelocities.scale(
                    calcAirDragVel(currentVelocity, dt) / currentVelocity
                );

                // next pos before collisions
                Translation3d nextPos = pos.plus(flightVelocities.scale(dt));
                Translation3d midPos = pos.plus(nextPos.minus(pos).scale(0.5));

                // ground interaction
                if(nextPos.getZ() <= kRadius){
                    flightVelocities = collideOnField(
                        flightVelocities, spinDirection, spinVel,
                        new Translation3d(0, 0, 1),
                        kFloorBounceFactor,
                        kFloorBounceVelFriction,
                        kStaticFrictionAccel,
                        dt
                    );
                    spinVel = 0;

                    pos = pos.plusZ(kRadius - pos.getZ());
                    nextPos = pos.plus(flightVelocities.scale(dt));
                    midPos = pos.plus(nextPos.minus(pos).scale(0.5));
                }
                // upper hub interaction
                // on walls
                Rotation2d upperHubIncline = FieldUtil.kUpperHubIncline;
                Translation3d upperGoalBottomVertex = new Translation3d(
                    0,
                    0,
                    FieldUtil.kVisionRingHeight -
                        (FieldUtil.kVisionRingDiameter/2.0 * Math.tan(
                            Math.abs(upperHubIncline.unaryMinus().getRadians())
                        ))
                )
                .plusXY(FieldUtil.kFieldCenter)
                // adjust point so that angle seen matching the incline is when the cargo touches the wall
                .plusZ(kRadius / Math.sin(Math.PI/2.0 - upperHubIncline.getRadians()));
                Translation3d hubWallNormal = new Translation3d(
                    1,
                    midPos.minus(upperGoalBottomVertex).getYaw(),
                    upperHubIncline.plus(new Rotation2d(Math.PI/2))
                );
                
                double centerDist = FieldUtil.kFieldCenter.getDistance(pos.getXY());
                // hit upper goal buttom
                if(centerDist <= FieldUtil.kUpperGoalBottomDiameter/2.0 + kRadius &&
                    nextPos.getZ() < FieldUtil.kUpperGoalBottomHeight + kRadius &&
                    pos.getZ() >= FieldUtil.kUpperGoalBottomHeight + kRadius
                ){
                    flightVelocities = collideOnField(
                        flightVelocities, spinDirection, spinVel,
                        new Translation3d(0, 0, 1),
                        kHubBounceFactor,
                        kHubBounceVelFriction,
                        kStaticFrictionAccel,
                        dt
                    );
                    spinVel = 0;

                    pos = pos.plusZ(FieldUtil.kUpperGoalBottomHeight + kRadius - pos.getZ());
                    nextPos = pos.plus(flightVelocities.scale(dt));
                    midPos = pos.plus(nextPos.minus(pos).scale(0.5));
                }
                
                double distIntoHubWall = Math.copySign(
                    pos.minus(upperGoalBottomVertex).proj(hubWallNormal).getDistance(),
                    pos.minus(upperGoalBottomVertex).dot(hubWallNormal.unaryMinus())
                );
                double nextDistIntoHubWall = Math.copySign(
                    nextPos.minus(upperGoalBottomVertex).proj(hubWallNormal).getDistance(),
                    nextPos.minus(upperGoalBottomVertex).dot(hubWallNormal.unaryMinus())
                );
                // hit upper goal wall
                if(nextPos.getZ() >= FieldUtil.kUpperGoalBottomHeight - kRadius/2 &&
                    nextPos.getZ() <= FieldUtil.kVisionRingHeight + kRadius/2 &&
                    nextDistIntoHubWall > 0 &&
                    nextDistIntoHubWall > distIntoHubWall &&
                    distIntoHubWall <= kRadius/2
                ){
                    flightVelocities = collideOnField(
                        flightVelocities, spinDirection, spinVel,
                        hubWallNormal,
                        kHubBounceFactor,
                        kHubBounceVelFriction,
                        kStaticFrictionAccel,
                        dt
                    );
                    spinVel = 0;

                    pos = pos.plus(hubWallNormal.scale(distIntoHubWall / hubWallNormal.getDistance()));
                    nextPos = pos.plus(flightVelocities.scale(dt));
                    midPos = pos.plus(nextPos.minus(pos).scale(0.5));
                }

                if(cargoNum == 1) SmartDashboard.putString("cargo1/vel", flightVelocities.toString());

                pos = nextPos;

                if(Math.abs(flightVelocities.getDistance()) <= kMinBounceVel) {
                    groundSeconds += dt;
                }
                if(groundSeconds > 3){
                    reset();
                }
            }
            else if(state == State.CONTROLLED){
                // cargo movement in intake/indexer
                double intakedDistance = intakeVelocity * dt;
                double indexedDistance = indexVelocity * dt;
                currentPathDistance += (currentPathDistance <= kIndexBeginDistance) ? 
                    intakedDistance
                    :
                    indexedDistance
                ;

                // sensors
                if(currentPathDistance >= kBotSensorBeginDistance &&
                        currentPathDistance <= kBotSensorEndDistance
                ) {
                    tripsBot = true;
                }
                else tripsBot = false;
                if(currentPathDistance >= kTopSensorBeginDistance) tripsTop = true;
                else tripsTop = false;

                // transition to shot
                if(currentPathDistance >= kPathDistance && shotVelocity > 0.1){
                    setState(State.FLIGHT);
                    setPos(shooterPos);
                    spinDirection = robotYaw;
                    spinVel = shotSpinVelocity;
                    tripsBot = false;
                    tripsTop = false;
                    
                    // cargo velocities based on shooter state
                    flightVelocities = calcShotVelocities(
                        shotVelocity,
                        shotPitch,
                        shotYaw,
                        robotSpeeds
                    );
                }
                else if(currentPathDistance < 0 && intakeVelocity < -0.1){
                    setState(State.FLIGHT);
                    setPos(intakePos);
                    spinDirection = robotYaw;
                    spinVel = -intakeVelocity;
                    tripsBot = false;
                    tripsTop = false;

                    // cargo velocities based on intake state
                    flightVelocities = calcShotVelocities(
                        -intakeVelocity,
                        new Rotation2d(),
                        shotYaw,
                        robotSpeeds
                    );
                }
                else{
                    setPos(intakePos.interpolate(shooterPos, currentPathDistance / kPathDistance));
                }
            }
            else{
                Translation3d relativeCargo = pos.minus(intakePos).rotateByYaw(robotYaw.unaryMinus());
                if(Math.abs(relativeCargo.getX()) < kIntakeXDist &&
                    Math.abs(relativeCargo.getY()) < kIntakeYDist &&
                    intakeVelocity > 0.1
                ){
                    setState(State.CONTROLLED);
                    setPos(intakePos);
                }
            }

            if(cargoNum == 1) SmartDashboard.putString("cargo1/pos", pos.toString());
        }

        public boolean getTripsBottom(){return tripsBot;}
        public boolean getTripsTop(){return tripsTop;}
    }

    //-----

    private double lastTime = Timer.getFPGATimestamp();

    private List<SimCargo> cargoList = new ArrayList<>();

    private final Field2d xyField;
    private final Field2d xzField;
    
    private final Supplier<Pose2d> getRobotPose;
    private final Supplier<ChassisSpeeds> getRobotSpeeds;
    
    private final DoubleSupplier getIntakeRPM;
    private final DoubleSupplier getIndexerRPM;
    private final Supplier<Shooter.State> getShooterState;

    private final BooleanConsumer bottomSensorSim;
    private final BooleanConsumer topSensorSim;

    public CargoSim(
            Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds,
            DoubleSupplier intakeRPM, DoubleSupplier indexerRPM,
            Supplier<Shooter.State> shooterState,
            BooleanConsumer bottomSensorSim,
            BooleanConsumer topSensorSim,
            Field2d xyField,
            Field2d xzField
    ){
        getRobotPose = robotPose;
        getRobotSpeeds = robotSpeeds;
        getIntakeRPM = intakeRPM;
        getIndexerRPM = indexerRPM;
        this.getShooterState = shooterState;
        this.bottomSensorSim = bottomSensorSim;
        this.topSensorSim = topSensorSim;
        this.xyField = xyField;
        this.xzField = xzField;

        placeAllianceCargo(false, cargoList);
        placeAllianceCargo(true, cargoList);
    }

    private void placeAllianceCargo(boolean flip, List<SimCargo> cargoListList){
        Translation2d fieldCenter = FieldUtil.kFieldCenter;
        Rotation2d offset = new Rotation2d(flip ? Math.PI : 0);
        // counted from top down (driver view: left to right)
        Translation2d opponent1 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(125.25)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d alliance1 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(147.75)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d opponent2 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(-167.25)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d alliance2 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(-144.75)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d alliance3 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(-99.75)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d opponent3 = new Translation2d(
            FieldUtil.kCargoPlacementDiameter/2.0,
            Rotation2d.fromDegrees(-77.25)
        ).rotateBy(offset).plus(fieldCenter);
        Translation2d terminalCargo = new Translation2d(
            -Units.inchesToMeters(282.08),
            -Units.inchesToMeters(117.725)
        ).rotateBy(offset).plus(fieldCenter);

        cargoListList.addAll(Arrays.asList(
            new SimCargo(opponent1),new SimCargo(opponent2),new SimCargo(opponent3),
            new SimCargo(alliance1),new SimCargo(alliance2),new SimCargo(alliance3),
            new SimCargo(terminalCargo)
        ));
    }

    public void update(){

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        Pose2d robotPose = getRobotPose.get();
        ChassisSpeeds robotSpeeds = getRobotSpeeds.get();
        double intakeRPM = getIntakeRPM.getAsDouble();
        double indexerRPM = getIndexerRPM.getAsDouble();
        Shooter.State shooterState = getShooterState.get();

        boolean bottomSensed = false;
        boolean topSensed = false;
        for(SimCargo cargo : cargoList){
            cargo.update(dt, robotPose, robotSpeeds, intakeRPM, indexerRPM, shooterState);
            // update indexer sensors
            
            bottomSensed = cargo.getTripsBottom() || bottomSensed;
            topSensed = cargo.getTripsTop() || topSensed;
        }

        bottomSensorSim.accept(bottomSensed);
        topSensorSim.accept(topSensed);

        xyField.getObject("Cargo").setPoses(
            cargoList.stream()
            .map((cargo)->new Pose2d(cargo.getPos().getXY(), new Rotation2d()))
            .collect(Collectors.toList())
        );
        xyField.getObject("Intake").setPose(robotPose.transformBy(new Transform2d(
            new Translation2d(kIntakeOffset, 0), new Rotation2d()
        )));

        xzField.setRobotPose(
            robotPose.getX(),
            kShooterHeight/2.0,
            robotPose.getRotation().getCos() > 0 ? new Rotation2d() : new Rotation2d(-1, 0)
        );
        xzField.getObject("Cargo").setPoses(
            cargoList.stream()
            .map((cargo)->new Pose2d(cargo.getPos().getXZ(), new Rotation2d()))
            .collect(Collectors.toList())
        );
        xzField.getObject("HubCorn").setPoses(
            new Pose2d(
                FieldUtil.kFieldCenter.getX() - FieldUtil.kVisionRingDiameter/2.0,
                FieldUtil.kVisionRingHeight,
                new Rotation2d()
            ),
            new Pose2d(
                FieldUtil.kFieldCenter.getX() - FieldUtil.kUpperGoalBottomDiameter/2.0,
                FieldUtil.kUpperGoalBottomHeight,
                new Rotation2d()
            ),
            new Pose2d(
                FieldUtil.kFieldCenter.getX() + FieldUtil.kUpperGoalBottomDiameter/2.0,
                FieldUtil.kUpperGoalBottomHeight,
                new Rotation2d()
            ),
            new Pose2d(
                FieldUtil.kFieldCenter.getX() + FieldUtil.kVisionRingDiameter/2.0,
                FieldUtil.kVisionRingHeight,
                new Rotation2d()
            )
        );
    }
}
