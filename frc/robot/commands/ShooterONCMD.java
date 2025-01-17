package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ShooterONCMD extends Command {
  /** Creates a new BananaIntakeControl. */
  Shooter m_Shooter;
 
  public ShooterONCMD(Shooter m_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Shooter = m_Shooter;
    addRequirements(m_Shooter);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Shooter.bananaFwdShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.bananaFwdShooter();
    
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
//       m_Shooter.bananaFwdShooter();
//     return false;
//   }
}