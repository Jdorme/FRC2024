// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BananaIntake;

public class BananaIntakeONOveride extends Command {
  /** Creates a new BananaIntakeControl. */
  BananaIntake m_bananaIntake;
 
  public BananaIntakeONOveride(BananaIntake m_bananaIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_bananaIntake = m_bananaIntake;
    addRequirements(m_bananaIntake);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_bananaIntake.bananaFwdIntakeOverRide();
    
   SmartDashboard.putBoolean(getName(), isScheduled());
  }

//  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      SmartDashboard.putBoolean(getName(), isScheduled());
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      SmartDashboard.putBoolean(getName(), isScheduled());
      
      
    return false;
  }
 }
