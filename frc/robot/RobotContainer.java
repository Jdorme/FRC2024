// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.commands.ArmIntakeNote;
import frc.robot.commands.ArmSpeaker;
import frc.robot.commands.ArmZero;
import frc.robot.commands.AutoTime;
import frc.robot.commands.BananaEjectCMD;
import frc.robot.commands.BananaIntakeCMD;
import frc.robot.commands.BananaIntakeONOveride;
import frc.robot.commands.BananaIntakeStopCMD;
import frc.robot.commands.BananaStopAll;
import frc.robot.commands.LiftDownCMD;
import frc.robot.commands.LiftHoldCMD;
import frc.robot.commands.ShooterOFF;
import frc.robot.commands.ShooterONCMD;
import frc.robot.commands.LiftUpCMD;

import frc.robot.commands.ArmAmp;
import frc.robot.commands.ArmFarSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BananaIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShoulderArmSubsystem;

import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.BananaEjectCMD;
public class RobotContainer {
  private double MaxSpeed = 6.4; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  


  // DoubleSupplier leftTrigger = 1;
  //   DoubleSupplier rightTrigger = 1;
  //   boolean buttonA = false;
  //       boolean buttonB = false;

 // private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final static CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  

  private final BananaIntake m_bananaIntake = new BananaIntake();
  Shooter m_Shooter = new Shooter();
  ShoulderArmSubsystem m_shoulderArmSubsystem = new ShoulderArmSubsystem();
  WristSubsystem m_wristSubsystem = new WristSubsystem();
  LiftSubsystem m_liftsubsystem = new LiftSubsystem();
  


  BananaIntakeStopCMD bananaintakestopcmd = new BananaIntakeStopCMD(m_bananaIntake, m_Shooter);
  BananaIntakeCMD bananaintakecmd = new BananaIntakeCMD(m_bananaIntake);
  BananaIntakeONOveride m_BananaIntakeONOveride = new BananaIntakeONOveride(m_bananaIntake);

  ShooterONCMD shooteroncmd = new ShooterONCMD(m_Shooter);
  ShooterOFF shooteroff= new ShooterOFF(m_Shooter);

  BananaEjectCMD m_bananaEjectCMD= new BananaEjectCMD(m_bananaIntake, m_Shooter);
  BananaStopAll m_bananaStopAll = new BananaStopAll(m_bananaIntake);

  ArmAmp m_armAmp = new ArmAmp(m_shoulderArmSubsystem, m_wristSubsystem);
  ArmIntakeNote m_armIntakeNote = new ArmIntakeNote(m_shoulderArmSubsystem, m_wristSubsystem);
  ArmZero m_zero = new ArmZero(m_shoulderArmSubsystem, m_wristSubsystem);
  ArmSpeaker m_armSpeaker = new ArmSpeaker(m_shoulderArmSubsystem, m_wristSubsystem);
  ArmFarSpeaker m_armFarSpeaker = new ArmFarSpeaker(m_shoulderArmSubsystem, m_wristSubsystem);

  LiftUpCMD m_liftUpCMD = new LiftUpCMD(m_liftsubsystem);
  LiftDownCMD m_liftDownCMD = new LiftDownCMD(m_liftsubsystem);
  LiftHoldCMD m_liftHoldCMD = new LiftHoldCMD(m_liftsubsystem);

  AutoTime m_autoTime = new AutoTime(m_Shooter, m_bananaIntake, m_shoulderArmSubsystem, m_wristSubsystem);

  Double rightTrigger = joystick.getRightTriggerAxis();
  Double leftTrigger = joystick.getLeftTriggerAxis();

  private final SendableChooser<Command> autoChooser;
            
  //public final BananaIntake leftTrigger = new BananaIntake(joystick.getLeftTriggerAxis());

  
  
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  
  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("TestAuto");
  
  private final Telemetry logger = new Telemetry(MaxSpeed);
                      //  private final BananaIntake rightTrigger1 = new BananaIntake(joystick.getRightTriggerAxis());

    //Trigger combinedTrigger = joystick.rightBumper().and(joystick.a());
  



  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(((joystick.getRightX())) * MaxAngularRate))); // Drive counterclockwise with negative X (left)

            
   //joystick.rightBumper().whileFalse(bananaintakestopcmd);

    // joystick.rightBumper().and(joystick.rightTrigger()).whileFalse(bananaintakestopcmd);

    //     joystick.rightBumper().and(joystick.rightTrigger()).onTrue(bananaintakestopcmd);


    joystick.rightBumper().whileTrue(m_BananaIntakeONOveride);
    //joystick.rightTrigger().onTrue(m_bananaEjectCMD);
       joystick.rightBumper().whileFalse(bananaintakestopcmd);  
         // joystick.rightTrigger().whileFalse(bananaintakestopcmd);

  


    // reset the field-centric heading on left bumper press
    joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    
   // m_ShoulderSubsystem.setDefaultCommand(shouldera);
    //m_bananaIntake.setDefaultCommand(bananaintakecmd);
     //joystick.rightBumper().onFalse(bananaintakestopcmd);
     joystick.leftBumper().onTrue(shooteroncmd);
     joystick.leftBumper().onFalse(shooteroff);


        joystick.pov(0).whileTrue(m_armSpeaker);
        joystick.pov(90).whileTrue(m_armAmp);
        joystick.pov(180).whileTrue(m_armIntakeNote);
        joystick.pov(270).whileTrue(m_zero);
      //  joystick.x().whileTrue(m_armFarSpeaker);

        joystick.a().whileTrue(m_bananaEjectCMD);
        joystick.a().whileFalse(bananaintakestopcmd);

        


        

        
        joystick.y().whileTrue(m_liftUpCMD);
        joystick.a().whileTrue(m_liftDownCMD);
        joystick.rightTrigger().and(joystick.leftTrigger()).equals(0);
        if(rightTrigger >= .5){
          joystick.rightTrigger().onTrue(m_liftUpCMD);
        }else if(rightTrigger < .5){
          joystick.rightTrigger().onFalse(m_liftHoldCMD);
        }else if(leftTrigger >= .5){
          joystick.rightTrigger().onTrue(m_liftUpCMD);
        }else if(leftTrigger < .5){
          joystick.rightTrigger().onFalse(m_liftHoldCMD);
        }


        //   joystick.rightTrigger().onTrue(m_liftUpCMD);
        //  joystick.rightTrigger().onFalse(m_liftHoldCMD);
        // joystick.leftTrigger().onTrue(m_liftDownCMD);
        // joystick.leftTrigger().onFalse(m_liftHoldCMD);

        

        //and(joystick.a()).onFalse(m_liftHoldCMD);

        // joystick.y().and(joystick.x()).onFalse(m_liftHoldCMD);
        
        // joystick.y()
        // .and(joystick.x())
        // .whileTrue(m_liftHoldCMD);

        //joystick.rightTrigger().and(joystick.leftTrigger()).whileFalse(m_liftHoldCMD);


        //joystick.leftBumper().onTrue(bananaintakecmd);
            //joystick.rightBumper().equals();

           // SmartDashboard.putNumber("distance1", m_encoder.getAbsolutePosition());
    //SmartDashboard.putBoolean(getName(), isScheduled())

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  public RobotContainer() {
  
    NamedCommands.registerCommand("liftUp", m_liftUpCMD);
   // NamedCommands.registerCommand("liftUp", m_liftUpCMD);

    NamedCommands.registerCommand("armAmp", m_armAmp);
    NamedCommands.registerCommand("armIntakeNote", m_armIntakeNote);
    NamedCommands.registerCommand("armZero", m_zero);
    NamedCommands.registerCommand("armSpeaker", m_armSpeaker);

    NamedCommands.registerCommand("shooterOn", shooteroncmd);
    NamedCommands.registerCommand("shooterOff", shooteroff);

    NamedCommands.registerCommand("autoTime", m_autoTime);

// SmartDashboard.getNumber("Encoder", Pigeon2  )
     // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("New Auto", autoChooser);
    


  
    configureBindings(); 


  }; 
    
  

//    chooser.addOption("scoredown", loadPathplannerTrajectory("scoredown",true));

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    }
}
