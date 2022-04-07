package frc.robot.subsystems.shooter;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.util.FieldUtil;

public final class ShotMap {

    private static final TreeMap<Double, ShotMapEntry> map = new TreeMap<>();
    public static final Shooter.State kFenderLow = new Shooter.State(1500, 25);
    private static double rpmOffset = 0;
    static {
        // high goal states at distance in inches
        map.put(
            // fender high shot
            Units.metersToInches(FieldUtil.kCenterToFenderDist+SwerveConstants.kRobotWidth/2.0),
            new ShotMapEntry(2800, 0, 1.5)
        );
        map.put(100.0, new ShotMapEntry(3000, 24, 1.5));
        map.put(200.0, new ShotMapEntry(3500, 35, 1.75));
        map.put(300.0, new ShotMapEntry(5000, 50, 2)); 
    }

    private static ShotMapEntry findEntry(double distanceMeters){
        double distanceInches = Units.metersToInches(distanceMeters);
        distanceInches = MathUtil.clamp(distanceInches, map.firstKey(), map.lastKey());
        // close side nearest map entry
        double closeDist = map.floorKey(distanceInches);
        // far side nearest map entry
        double farDist = map.ceilingKey(distanceInches);
        double closeToFar = farDist - closeDist;
        double closeToTarget = distanceInches - closeDist;

        if(closeToFar == 0) return map.get(closeDist);
        return map.get(closeDist).interpolate(map.get(farDist), closeToTarget / closeToFar).plus(new ShotMapEntry(rpmOffset, 0, 0));
    }
    /**
     * Finds shooter state appropriate for shooting into the high goal at the given distance
     * @param distanceMeters Distance in meters from the center of the hub
     */
    public static Shooter.State find(double distanceMeters){
        return findEntry(distanceMeters).state;
    }
    /**
     * Finds the time-of-flight in seconds for the cargo if shot from the given distance
     * with the corresponding shooter state.
     * @param distanceMeters Distance in meters from the center of the hub
     * @return Time-of-flight of the cargo in seconds (exit shooter to contact hub)
     */
    public static double findToF(double distanceMeters){
        return findEntry(distanceMeters).tof;
    }
    public static void setRPMOffset(double rpm){
        rpmOffset = rpm;
    }

    private static class ShotMapEntry implements Interpolatable<ShotMapEntry>{
        public final Shooter.State state;
        public final double tof;

        /**
         * A shooter state with an additional time-of-flight for
         * estimating cargo speed.
         * @param tof Time-of-flight seconds at given distance
         */
        public ShotMapEntry(double rpm, double hoodMM, double tof){
            this.state = new Shooter.State(rpm, hoodMM);
            this.tof = tof;
        }
        public ShotMapEntry plus(ShotMapEntry other){
            Shooter.State newState = other.state.plus(state);
            return new ShotMapEntry(newState.rpm, newState.hoodMM,tof);
        }

        @Override
        public ShotMapEntry interpolate(ShotMapEntry endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                Shooter.State newState = state.interpolate(endValue.state, t);
                return new ShotMapEntry(
                    newState.rpm,
                    newState.hoodMM,
                    MathUtil.interpolate(tof, endValue.tof, t)
                );
            }
        }
    }
}
