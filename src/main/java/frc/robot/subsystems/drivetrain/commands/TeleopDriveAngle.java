// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TeleopDriveAngle extends CommandBase {
    
    private final OCXboxController controller;
    private final SwerveDrive drivetrain;

    // magnitude of minimum angle joystick command
    private final double kAngleMagnitudeDeadband = 0.12;
    private Rotation2d lastTargetRotation = new Rotation2d();

    public TeleopDriveAngle(OCXboxController controller, SwerveDrive drivetrain) {
        this.controller = controller;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        controller.resetLimiters();
        drivetrain.resetPathController(); // reset theta profile
        lastTargetRotation = drivetrain.getHeading();
    }
    
    @Override
    public void execute() {
        // convert joysticks to field-axes
        double xAxis = controller.getRightY();
        double yAxis = controller.getRightX();
        Rotation2d targetRotation = new Rotation2d(xAxis, yAxis);
        // deadband input
        double magnitude = Math.hypot(xAxis, yAxis);
        if(magnitude < kAngleMagnitudeDeadband){
            targetRotation = lastTargetRotation;
        }
        else{
            lastTargetRotation = targetRotation;
        }

        drivetrain.drive(
            controller.getForward() * drivetrain.getMaxLinearVelocityMeters(),
            controller.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
            targetRotation,
            true // this may cause inconsistent angle accuracy versus autonomous
        );
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
