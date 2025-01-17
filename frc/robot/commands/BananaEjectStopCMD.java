// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BananaIntake;

public class BananaEjectStopCMD extends Command {
  /** Creates a new BananaEjectStopCMD. */
  BananaEjectCMD m_BananaEjectCMD;


  public BananaEjectStopCMD( BananaEjectCMD m_BananaEjectCMD) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_BananaEjectCMD = m_BananaEjectCMD;
    addRequirements(m_BananaEjectCMD);
 
  }

  private void addRequirements(BananaEjectCMD m_BananaEjectCMD2) {


  }

  /*  Called when the command is initially scheduled.
  @Override
  public void initialize() {} */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  m_BananaEjectCMD.doNothingEject();
    
    SmartDashboard.putBoolean(getName(), isScheduled());
  }

  /*  Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }*/
}
