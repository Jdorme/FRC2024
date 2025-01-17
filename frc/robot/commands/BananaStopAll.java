package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BananaIntake;

public class BananaStopAll extends Command {
  /** Creates a new BananaIntakeControl. */
  BananaIntake m_bananaIntake;
 
  public BananaStopAll(BananaIntake m_bananaIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_bananaIntake = m_bananaIntake;
    addRequirements(m_bananaIntake);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_bananaIntake.doNothingIntake();
  //  m_bananaIntake.doNothingShooter();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_bananaIntake.doNothingIntake();
  //  m_bananaIntake.doNothingShooter();
    
   SmartDashboard.putBoolean(getName(), isScheduled());
  }

 // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//       SmartDashboard.putBoolean(getName(), isScheduled());
//   }

//  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//       SmartDashboard.putBoolean(getName(), isScheduled());
//       m_bananaIntake.doNothingIntake();
//     return false;
//   }
}