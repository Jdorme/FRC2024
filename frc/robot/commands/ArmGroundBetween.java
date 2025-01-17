// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmGroundBetween extends Command {
  /** Creates a new ShoulderPositionA. */
  ShoulderArmSubsystem m_shoulderArmSubsystem;
  WristSubsystem m_wristSubsystem;

  public ArmGroundBetween(ShoulderArmSubsystem m_shoulderArmSubsystem, WristSubsystem m_wristSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shoulderArmSubsystem = m_shoulderArmSubsystem;
    this.m_wristSubsystem = m_wristSubsystem;
    addRequirements(m_shoulderArmSubsystem, m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_wristSubsystem.inBetween();
   m_wristSubsystem.enable();
  //  m_shoulderArmSubsystem.displaySetpoint();
  //  m_wristSubsystem.displaySetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
