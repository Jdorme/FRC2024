// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.BananaIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShoulderArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoTime extends Command {
  Timer timer = new Timer();
  Shooter m_Shooter;
  BananaIntake m_bananaIntake;
  ShoulderArmSubsystem m_shoulderArmSubsystem;
  WristSubsystem m_wristSubsystem;
  String completed = "false"; 
  Boolean timerStarted = false;
  /** Creates a new Timer. */
  public AutoTime(Shooter m_Shooter, BananaIntake m_bananaIntake, ShoulderArmSubsystem m_shoulderArmSubsystem, WristSubsystem m_wristSubsystem) {
    this.m_Shooter = m_Shooter;
    this.m_bananaIntake = m_bananaIntake;
    this.m_shoulderArmSubsystem = m_shoulderArmSubsystem;
    this.m_wristSubsystem = m_wristSubsystem;
    addRequirements(m_Shooter, m_bananaIntake,m_shoulderArmSubsystem, m_wristSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(timerStarted == false){
    timer.reset();
    timer.start();
    timerStarted = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Current Auto", completed);
    SmartDashboard.putNumber("Time", timer.get());
    if (timer.get()<3 && completed == "false"){
      m_shoulderArmSubsystem.intakeNote();
      m_wristSubsystem.intakeNote();
      m_shoulderArmSubsystem.enable();
      m_wristSubsystem.enable();
      completed = "ground";
    }else if (timer.get()<5 && completed == "ground"){
    m_Shooter.bananaFwdShooter();
    completed = "shooterSpin";
    }else if (timer.get()>5.1 && completed == "shooterSpin"){
      m_bananaIntake.bananaFwdIntakeOverRide();
      completed = "shootNote";
    }else if (timer.get() > 8 && completed == "shootNote"){
      m_bananaIntake.doNothingIntake();
      m_Shooter.doNothingShooter();
      m_shoulderArmSubsystem.zero();
      m_wristSubsystem.zero();
      m_shoulderArmSubsystem.enable();
      m_wristSubsystem.enable();
      completed = "zero";
    }else if(timer.get() > 11 && completed == "zero"){
      
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() == 11.0){
         return true;
    } else { return false;
    }
  }
}
