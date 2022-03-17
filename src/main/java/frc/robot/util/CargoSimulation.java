package frc.robot.util;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class CargoSimulation {
    
    private static class SimCargo {
        private static final double dt = 0.02;
        private static int totalCargo = 0;

        private final int cargoNum;
        public static final double kGravityAccel = -9.8;
        public static final double kCargoRadius = Units.inchesToMeters(9.5 / 2.0);
        public static final double kIntakeOffset = Units.inchesToMeters(20);
        public static final double kIntakeHeight = Units.inchesToMeters(kCargoRadius);
        public static final double kIntakeDistance = kCargoRadius + Units.inchesToMeters(3); // cargo closer than this will become intaked
        public static final double kShooterHeight = Units.inchesToMeters(24);
        
        public static final double kBotSensorBeginDistance = Units.inchesToMeters(6);
        public static final double kIndexBeginDistance = Units.inchesToMeters(11); // 25% intake, 75% index
        public static final double kBotSensorEndDistance = Units.inchesToMeters(17);
        public static final double kTopSensorBeginDistance = Units.inchesToMeters(29);
        
        public static final double kPathDistance = Math.hypot(kIntakeOffset, kShooterHeight);

        public final Translation3d resetPos;

        // distinguish idle, intaking, and shooting
        enum State{
            IDLE,
            CONTROLLED,
            FLIGHT
        }
        private State state = State.IDLE;

        // controlled variables
        private double currentPathDistance = 0;

        // flight variables
        private Rotation2d flightYaw = new Rotation2d();
        private Rotation2d flightPitch = new Rotation2d();
        private double flightMPS = 0;
        private double flightSeconds = 0;

        private Translation3d pos;
        // indexer sensors
        private boolean tripsBot = false;
        private boolean tripsTop = false;

        public SimCargo(Translation2d resetPos){
            this.resetPos = new Translation3d(resetPos.getX(), resetPos.getY(), kCargoRadius);
            pos = this.resetPos;

            totalCargo++;
            this.cargoNum = totalCargo;
        }

        public void setPos(Translation3d pos){this.pos = pos;}
        public void setState(State state){this.state = state;}
        
        public void reset(){
            setPos(resetPos);
            state = State.IDLE;
            currentPathDistance = 0;
            flightSeconds = 0;
            tripsBot = false;
            tripsTop = false;
        }

        public State getState(){return state;}
        public Translation3d getPos(){return pos;}

        public void update(Pose2d robotPose, double intakeRPM, double indexerRPM, Shooter.State shooterState){
            Translation3d intakePos = new Translation3d(0, 0, kIntakeHeight).plusXY(
                robotPose.transformBy(new Transform2d(
                    new Translation2d(kIntakeOffset, 0), new Rotation2d()
                )).getTranslation()
            );
            Translation3d shooterPos = new Translation3d(0, 0, kShooterHeight).plusXY(robotPose.getTranslation());

            double intakeVelocity = intakeRPM / 60 * Units.inchesToMeters(4) * Math.PI * 0.5;
            double indexVelocity = indexerRPM / 60 * Units.inchesToMeters(4) * Math.PI * 0.25;

            double shotVelocity = shooterState.rpm / 60 * Units.inchesToMeters(4) * Math.PI * 0.55;
            Rotation2d shotPitch = Rotation2d.fromDegrees(
                MathUtil.interpolate(85, 40, shooterState.hoodMM/ShooterConstants.kServoLengthMM)
            );

            if(state == State.FLIGHT){
                flightSeconds += dt;

                // initial x/y translation velocities
                double vx = flightMPS * flightPitch.getCos() * flightYaw.getCos();
                double vy = flightMPS * flightPitch.getCos() * flightYaw.getSin();
                // z velocity after gravity
                double vz = flightMPS * flightPitch.getSin() + kGravityAccel*flightSeconds;

                pos = pos.plus(new Translation3d(vx * dt, vy * dt, vz * dt));

                // hit ground and stop
                if(pos.getZ() <= kCargoRadius){
                    pos = pos.plus(new Translation3d(0, 0, kCargoRadius - pos.getZ()));
                    flightMPS = Math.max(0, flightMPS*0.98 - 0.02);
                }

                if(flightSeconds > 5){
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
                if(currentPathDistance >= kPathDistance && shotVelocity > 0){
                    setState(State.FLIGHT);
                    setPos(shooterPos);
                    tripsBot = false;
                    tripsTop = false;
                    flightMPS = shotVelocity;
                    flightPitch = shotPitch;
                    flightYaw = robotPose.getRotation().plus(new Rotation2d(Math.PI));
                }
                else if(currentPathDistance < 0 && intakeVelocity < 0){
                    setState(State.FLIGHT);
                    setPos(intakePos);
                    flightMPS = -intakeVelocity;
                    flightPitch = new Rotation2d(0.01);
                    flightYaw = robotPose.getRotation();
                }
                else{
                    setPos(intakePos.interpolate(shooterPos, currentPathDistance / kPathDistance));
                }
            }
            else{
                double dist = pos.getDistance(intakePos);
                if(dist < kIntakeDistance){
                    setState(State.CONTROLLED);
                    setPos(intakePos);
                }
            }
        }

        public boolean getTripsBottom(){return tripsBot;}
        public boolean getTripsTop(){return tripsTop;}
    }

    //-----

    private List<SimCargo> cargoList = new ArrayList<>();
    private final Field2d xyField;
    private final Field2d xzField;
    
    private final Supplier<Pose2d> getPose;
    
    private final DoubleSupplier getIntakeRPM;
    private final DoubleSupplier getIndexerRPM;
    private final Supplier<Shooter.State> getShooterState;

    private final BooleanConsumer bottomSensorSim;
    private final BooleanConsumer topSensorSim;

    public CargoSimulation(
        Supplier<Pose2d> pose,
        DoubleSupplier intakeRPM, DoubleSupplier indexerRPM,
        Supplier<Shooter.State> shooterState,
        BooleanConsumer bottomSensorSim,
        BooleanConsumer topSensorSim,
        Field2d xyField,
        Field2d xzField
        ){
        getPose = pose;
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
        Pose2d robotPose = getPose.get();
        double intakeRPM = getIntakeRPM.getAsDouble();
        double indexerRPM = getIndexerRPM.getAsDouble();
        Shooter.State shooterState = getShooterState.get();

        boolean bottomSensed = false;
        boolean topSensed = false;
        for(SimCargo cargo : cargoList){
            cargo.update(robotPose, intakeRPM, indexerRPM, shooterState);
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
        xzField.setRobotPose(
            robotPose.getX(),
            SimCargo.kShooterHeight/2.0,
            robotPose.getRotation().getCos() > 0 ? new Rotation2d() : new Rotation2d(-1, 0)
        );
        xzField.getObject("Cargo").setPoses(
            cargoList.stream()
            .map((cargo)->new Pose2d(cargo.getPos().getXZ(), new Rotation2d()))
            .collect(Collectors.toList())
        );
        xyField.getObject("Intake").setPose(robotPose.transformBy(new Transform2d(
            new Translation2d(SimCargo.kIntakeOffset, 0), new Rotation2d()
        )));
    }
}
