// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class Vision extends SubsystemBase {

    private Limelight limelight = new Limelight(
        kCameraOffset,
        kCameraHeight,
        kCameraPitch.getDegrees(),
        FieldUtil.kFieldCenter,
        FieldUtil.kVisionRingHeight,
        kLatencyMs
    );
    public Vision() {
        limelight.nuke();
    }
    
    @Override
    public void periodic() {
        // 0 side by side, 1 main, 2 driver cam
        limelight.setStreamMode(2);
        if(DriverStation.isDisabled()){
            // 0 default, 1 off, 2 blink, 3 on
            limelight.setLedMode(1);
        }
        else{
            // 0 default, 1 off, 2 blink, 3 on
            limelight.setLedMode(0);
        }
    }
    public double getDistance(){
        return limelight.getRobotDistance() + FieldUtil.kVisionRingDiameter;
    }
    public Pose2d getRobotPose(Rotation2d robotHeading){
        Translation2d relativeTranslation = new Translation2d(
            getDistance(),
            robotHeading
        ).plus(FieldUtil.kFieldCenter);
        return new Pose2d(relativeTranslation, robotHeading);
    }
    public boolean getHasTarget(){
        return limelight.getHasTarget();
    }
    public Rotation2d getTargetYaw(){
        return limelight.getRobotYaw();
    }
    public Rotation2d getTargetPitch(){
        return Rotation2d.fromDegrees(limelight.getTy());
    }
    public double getLatencySeconds(){
        return limelight.getLatencySeconds();
    }
    public void log(){
        SmartDashboard.putNumber("Vision/YawDegrees", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Vision/Distance", getDistance());
    }
}
