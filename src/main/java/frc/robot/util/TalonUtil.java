package frc.robot.util;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Utility methods for using the Phoenix library with Falcon 500s
 */
public class TalonUtil {

    /**
     * Configures the status frame periods of given motors
     */
    public static void configStatusNormal(WPI_TalonFX... motors){
        for(WPI_TalonFX motor : motors){
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 50);
        }
    }


    //Conversions yuck
    public static double positionToRotations(double nativePosition, double motorRotationsPerMechanismRotation){
        return nativePosition / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double positionToDegrees(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 360;
    }
    public static double positionToRadians(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double positionToMeters(double nativePosition, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToPosition(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToPosition(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToPosition(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToPosition(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToPosition(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }

    public static double velocityToRotations(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return nativeVelocity * 10 / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double velocityToDegrees(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 360;
    }
    public static double velocityToRadians(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double velocityToMeters(double nativeVelocity, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToVelocity(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 / 10 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToVelocity(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToVelocity(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToVelocity(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToVelocity(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }
}
