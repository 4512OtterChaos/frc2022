// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Helper methods for turning vision data into field measurements.
 */
public final class VisionUtil {    
    /**
    * Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html Estimates
    * range to a target using the target's elevation. This method can produce more stable results
    * than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
    * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
    * for there to exist a height differential between goal and camera. The larger this differential,
    * the more accurate the distance estimate will be.
    *
    * <p>Units can be converted using the {@link edu.wpi.first.math.util.Units} class.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitch The pitch of the camera from the horizontal plane.
    *     Positive values up.
    * @param targetPitch The pitch of the target in the camera's lens. Positive
    *     values up.
    * @return The estimated distance to the target in meters.
    */
    public static double calculateDistanceToTarget(
            double cameraHeightMeters,
            double targetHeightMeters,
            Rotation2d cameraPitch,
            Rotation2d targetPitch
        ) {
        return (targetHeightMeters - cameraHeightMeters)
            / Math.tan(cameraPitch.getRadians() + targetPitch.getRadians());
    }
    
    /**
    * Estimate the {@link Translation2d} of the target relative to the camera.
    *
    * @param targetDistanceMeters The distance to the target in meters.
    * @param targetYaw The observed yaw of the target.
    * @return The target's camera-relative translation.
    */
    public static Translation2d estimateCameraToTargetTranslation(
            double targetDistanceMeters, Rotation2d targetYaw
        ) {
        return new Translation2d(targetYaw.getCos() * targetDistanceMeters, targetYaw.getSin() * targetDistanceMeters);
    }
    
    /**
    * Estimate the position of the robot in the field.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitch The pitch of the camera from the horizontal plane.
    *     Positive values up.
    * @param targetPitch The pitch of the target in the camera's lens. Positive
    *     values up.
    * @param targetYaw The observed yaw of the target. Note that this *must* be CCW-positive, and
    *     Photon returns CW-positive.
    * @param robotHeading The current robot heading, likely from odometry.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            double cameraHeightMeters,
            double targetHeightMeters,
            Rotation2d cameraPitch,
            Rotation2d targetPitch,
            Rotation2d targetYaw,
            Rotation2d robotHeading,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot
        ) {
        return estimateFieldToRobot(
            estimateCameraToTarget(
                estimateCameraToTargetTranslation(
                    calculateDistanceToTarget(
                        cameraHeightMeters, targetHeightMeters, cameraPitch, targetPitch
                    ),
                    targetYaw
                ),
                fieldToTarget,
                robotHeading
            ),
            fieldToTarget,
            cameraToRobot
        );
    }
    /**
    * Estimate the position of the robot in the field.
    *
    * @param targetDistanceMeters Estimated distance in meters to the target from the camera
    * @param targetYaw The observed yaw of the target. Note that this *must* be CCW-positive, and
    *     Photon returns CW-positive.
    * @param robotHeading The current robot heading, likely from odometry.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            double targetDistanceMeters,
            Rotation2d targetYaw,
            Rotation2d robotHeading,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot
        ) {
        return estimateFieldToRobot(
            estimateCameraToTarget(
                estimateCameraToTargetTranslation(
                    targetDistanceMeters,
                    targetYaw
                ),
                fieldToTarget,
                robotHeading
            ),
            fieldToTarget,
            cameraToRobot
        );
    }
    /**
    * Estimates the pose of the robot in the field coordinate system, given the position of the
    * target relative to the camera, the target relative to the field, and the robot relative to the
    * camera.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            Transform2d cameraToTarget, Pose2d fieldToTarget, Transform2d cameraToRobot
        ) {
        return estimateFieldToCamera(cameraToTarget, fieldToTarget).transformBy(cameraToRobot);
    }
    
    /**
    * Estimates a {@link Transform2d} that maps the camera position to the target position, using the
    * robot's gyro. Note that the gyro angle provided *must* line up with the field coordinate system
    * -- that is, it should read zero degrees when pointed towards the opposing alliance station, and
    * increase as the robot rotates CCW.
    *
    * @param cameraToTargetTranslation A Translation2d that encodes the x/y position of the target
    *     relative to the camera.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param robotHeading The current robot heading, likely from odometry.
    * @return A Transform2d that takes us from the camera to the target.
    */
    public static Transform2d estimateCameraToTarget(
            Translation2d cameraToTargetTranslation, Pose2d fieldToTarget, Rotation2d robotHeading
        ) {
        // This pose maps our camera at the origin out to our target, in the robot
        // reference frame
        // The translation part of this Transform2d is from the above step, and the
        // rotation uses our robot's
        // gyro.
        return new Transform2d(
            cameraToTargetTranslation,
            robotHeading.unaryMinus().minus(fieldToTarget.getRotation())
        );
    }
    
    /**
    * Estimates the pose of the camera in the field coordinate system, given the position of the
    * target relative to the camera, and the target relative to the field. This *only* tracks the
    * position of the camera, not the position of the robot itself.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @return The position of the camera in the field.
    */
    public static Pose2d estimateFieldToCamera(Transform2d cameraToTarget, Pose2d fieldToTarget) {
        var targetToCamera = cameraToTarget.inverse();
        return fieldToTarget.transformBy(targetToCamera);
    }
}