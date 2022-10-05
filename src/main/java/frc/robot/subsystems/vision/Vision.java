// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import frc.robot.util.VisionUtil;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class Vision extends SubsystemBase {

    private final Limelight limelight = new Limelight();
    
    private double lastLatency = 0;

    public Vision() {
    }
    
    @Override
    public void periodic() {
        // 0 side by side, 1 main, 2 driver cam
        limelight.setStreamMode(1);
        if(DriverStation.isDisabled()){
            // 0 default, 1 off, 2 blink, 3 on
            limelight.setLedMode(1);
        }
        else{
            // 0 default, 1 off, 2 blink, 3 on
            limelight.setLedMode(3);
        }
    }

    /**
     * Estimates XY Distance in meters from the camera to the target (including 
     * distance to the hub center).
     */
    public double getCamDistanceMeters(){
        return VisionUtil.calculateDistanceToTarget(
            kCameraHeight,
            FieldUtil.kVisionRingHeight,
            kCameraPitch,
            getTargetPitch(),
            getTargetYaw()
        ) + (FieldUtil.kVisionRingDiameter / 2.0); // + vision ring -> hub center
    }

    /**
     * Estimates the transform from the current robot pose to the target pose.
     */
    public Transform2d getRobotToTarget(Rotation2d robotHeading){
        return kCameraOffset.plus(VisionUtil.estimateCameraToTarget(
            VisionUtil.estimateCameraToTargetTranslation(
                getCamDistanceMeters(),
                getTargetYaw()
            ),
            new Pose2d(FieldUtil.kFieldCenter, new Rotation2d()),
            robotHeading
        ));
    }
    /**
     * Estimates the robot's pose on the field given the distance to the target from the
     * camera, the robot's heading, and the target's translation on the field.
     * The robot is found relative to the target and then placed on the field using
     * the target's field translation.
     */
    public Pose2d getRobotPose(Rotation2d robotHeading){
        return VisionUtil.estimateFieldToRobot(
            getCamDistanceMeters(),
            getTargetYaw(),
            robotHeading,
            new Pose2d(FieldUtil.kFieldCenter, new Rotation2d()),
            kCameraOffset
        );
    }

    /**
     * If the camera currently has an acquired target.
     * This should be used whenever attempting to get vision data to ensure
     * it is applicable.
     */
    public boolean getHasTarget(){
        double latency = getLatencySeconds();
        // failsafe when NT is lost
        boolean wasUpdated = latency != lastLatency;
        lastLatency = latency;
        return limelight.getHasTarget();
    }

    /**
     * Target yaw reported by camera (lens -> target)
     */
    public Rotation2d getTargetYaw(){
        return Rotation2d.fromDegrees(-limelight.getTx());
    }
    /**
     * Target yaw incorporating camera offset to get robot center -> target.
     * Note this equivalent to the angle error between the robot's current camera yaw
     * and the yaw that would face the camera to target.
     */
    public Translation2d getRobotToTargetTranslation(){
        // get target relative to camera with offset corrected (robot center)
        return new Translation2d(
            getCamDistanceMeters(),
            getTargetYaw()
        ).minus(kCameraOffset.getTranslation()).unaryMinus();
    }

    public Translation2d getFilteredRobotToTargetTranslation(){
        // get target relative to camera with offset corrected (robot center)
        return new Translation2d(
            VisionUtil.calculateDistanceToTarget(
            kCameraHeight,
            FieldUtil.kVisionRingHeight,
            kCameraPitch,
            Rotation2d.fromDegrees(limelight.getFilteredTy()),
            Rotation2d.fromDegrees(-limelight.getFilteredTx())
        ) + (FieldUtil.kVisionRingDiameter / 2.0),
            Rotation2d.fromDegrees(-limelight.getFilteredTx())
        ).minus(kCameraOffset.getTranslation()).unaryMinus();
    }
    /**
     * Target pitch reported by camera (lens -> target)
     */
    public Rotation2d getTargetPitch(){
        return Rotation2d.fromDegrees(limelight.getTy());
    }

    public double getLatencySeconds(){
        return limelight.getLatencySeconds();
    }
    public void resetFilter(){
        limelight.resetFilter();
    }
    public void log(){
        SmartDashboard.putNumber("Vision/YawDegrees", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Vision/Distance", getCamDistanceMeters());
    }
}
