package frc.robot.common;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Shooter;

public final class ShotMap {

    private final TreeMap<Double, Shooter.State> map = new TreeMap<>();

    public ShotMap(Shooter shooter){
        map.put(10.0, shooter.new State(2000, 0));
        map.put(100.0, shooter.new State(3000, 20));
        map.put(240.0, shooter.new State(5000, 60));
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
