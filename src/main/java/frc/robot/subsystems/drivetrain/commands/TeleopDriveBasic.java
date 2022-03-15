// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * Basic tele-op velocity control using a controller.
 */
public class TeleopDriveBasic extends CommandBase {

    private final OCXboxController controller;
    private final SwerveDrive drivetrain;
    public TeleopDriveBasic(OCXboxController controller, SwerveDrive drivetrain) {
        this.controller = controller;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        controller.resetLimiters();
    }
    
    @Override
    public void execute() {
        drivetrain.drive(
            controller.getForward() * drivetrain.getMaxLinearVelocityMeters(),
            controller.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
            controller.getTurn() * drivetrain.getMaxAngularVelocityRadians(),
            true
        );
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
