// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer container;

    private Command autoCommand;

    private Timer disableTimer = new Timer();

    @Override
    public void robotInit() {
        container = new RobotContainer();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        
        container.periodic();
        container.log();
    }
    
    @Override
    public void autonomousInit() {
        container.setAllBrake(true);

        autoCommand = container.getAutoCommand();

        if(autoCommand != null){
            autoCommand.schedule();
        }
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        container.setAllBrake(true);

        if(autoCommand != null){
            autoCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {
        disableTimer.reset();
        disableTimer.start();
        
        container.disable();
    }
    
    @Override
    public void disabledPeriodic() {
        // coast motors in disabled mode so the robot can be moved
        if(disableTimer.hasElapsed(2)){
            disableTimer.stop();
            disableTimer.reset();

            container.setAllBrake(false);
        }
    }

    @Override
    public void simulationInit(){
    }
    @Override
    public void simulationPeriodic(){
    }
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}
}
