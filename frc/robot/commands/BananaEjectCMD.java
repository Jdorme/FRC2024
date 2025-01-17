// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BananaIntake;
import frc.robot.subsystems.Shooter;

public class BananaEjectCMD extends Command {
  /** Creates a new BananaIntakeControl. */
  Shooter m_shooter;
  BananaIntake m_bananaIntake;
  DoubleSupplier leftTrigger;
  DoubleSupplier rightTrigger;
  BooleanSupplier buttonA;
  BooleanSupplier buttonB;
  public BananaEjectCMD(BananaIntake m_bananaIntake, Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_bananaIntake = m_bananaIntake;
    this.m_shooter = m_shooter;
    
    addRequirements(m_bananaIntake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_bananaIntake.bananaBwd();
    m_shooter.bananaBwd();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       m_bananaIntake.bananaBwd();
       m_shooter.bananaBwd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_bananaIntake.doNothing();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // public void doNothingEject() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'doNothingEject'");
  // }
}
