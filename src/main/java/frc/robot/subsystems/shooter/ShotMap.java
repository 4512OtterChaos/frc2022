package frc.robot.subsystems.shooter;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;

public final class ShotMap {

    private final TreeMap<Double, Shooter.State> map = new TreeMap<>();

    public ShotMap(Shooter shooter){
        map.put(70.0, shooter.new State(3000, 10));
        map.put(200.0, shooter.new State(3500, 25));
        map.put(350.0, shooter.new State(5000, 50));
    }

    /**
     * Finds shooter state appropriate for given distance
     * @param distanceInches Distance in inches from the center of the hub
     */
    public Shooter.State find(double distanceInches){
        distanceInches = MathUtil.clamp(distanceInches, map.firstKey(), map.lastKey());
        // close side nearest map entry
        double closeDist = map.floorKey(distanceInches);
        // far side nearest map entry
        double farDist = map.ceilingKey(distanceInches);
        double closeToFar = farDist - closeDist;
        double closeToTarget = distanceInches - closeDist;

        if(closeToFar == 0) return map.get(closeDist);
        return map.get(closeDist).interpolate(map.get(farDist), closeToTarget / closeToFar);
    }
}
