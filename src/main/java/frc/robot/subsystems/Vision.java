// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Limelight;
import frc.robot.util.FieldUtil;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */
    private Limelight limelight = new Limelight(
        new Translation2d(Units.inchesToMeters(-12.5), 0),
        Units.inchesToMeters(26.5),
        37.5,
        FieldUtil.kFieldCenter,
        FieldUtil.kVisionRingHeight,
        40
    );
    public Vision() {
        limelight.nuke();

    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public double getDistance(){
        return limelight.getTrigDistance();
    }
    public boolean getHasTarget(){
        return limelight.getHasTarget();
    }
    public Rotation2d getYaw(){
        return Rotation2d.fromDegrees(limelight.getTx());
    }
    public Rotation2d getPitch(){
        return Rotation2d.fromDegrees(limelight.getTy());
    }
    public double getLatencySeconds(){
        return limelight.getLatencySeconds();
    }
    public void log(){
        SmartDashboard.putNumber("Vision/Distance", getDistance());
    }
}
