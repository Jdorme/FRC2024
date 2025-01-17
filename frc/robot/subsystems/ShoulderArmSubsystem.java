// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.WristConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ShoulderArmSubsystem extends ProfiledPIDSubsystem {
  public String currentSetpoint;
  private final TalonFX m_motor = new TalonFX(34, "rio");
  public static Encoder encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        ShoulderConstants.kSVolts, ShoulderConstants.kGVolts,
        ShoulderConstants.kVVoltSecondPerRad, ShoulderConstants.kAVoltSecondSquaredPerRad);

        Encoder encoder2 = WristSubsystem.encoder;

  /** Create a new ArmSubsystem. */
  public ShoulderArmSubsystem() {
    super(
        new ProfiledPIDController(
            2,
            0,
            0,
            new TrapezoidProfile.Constraints(
              ShoulderConstants.kMaxVelocityRadPerSecond,
              ShoulderConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    encoder.setDistancePerPulse(ShoulderConstants.kEncoderDistancePerPulse);
    
    // Start arm at rest in neutral position
    setGoal(ShoulderConstants.kArmOffsetRads);

// Configures the encoder to consider itself stopped when its rate is below 10
encoder.setMinRate(10);
  }

  // public void goToPositionA(TrapezoidProfile.State setpoint) {
  //   // Calculate the feedforward from the sepoint
  //   double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   // Add the feedforward to the PID output to get the motor output
  //   m_motor.setVoltage(0);
  //   SmartDashboard.putNumber("encoder", encoder.getDistance());
  // }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(-output);
    SmartDashboard.putNumber("Shoulder Encoder", encoder.getDistance());
    SmartDashboard.putString("Shoulder Setpoint", currentSetpoint);
    
  }
  @Override
public void periodic() {
 if((encoder2.getDistance() <= -20 + WristConstants.kArmOffsetRads || encoder2.getDistance() >= -15 + WristConstants.kArmOffsetRads) && currentSetpoint == "intakeNoteTransitionNear"){
    setGoal(WristConstants.intakeNoteSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "intakeNote";
 }else if ((encoder2.getDistance() <= -10 + WristConstants.kArmOffsetRads || encoder2.getDistance() >= -8 + WristConstants.kArmOffsetRads) && currentSetpoint == "intakeNoteTransitionFar"){
    setGoal(WristConstants.intakeNoteSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "intakeNote";
 }


    super.periodic();
}

  @Override
  public double getMeasurement() {
    return encoder.getDistance() + ShoulderConstants.kArmOffsetRads; }

    public void ampScore(){
     setGoal(ShoulderConstants.ampScoreSetpoint);
     currentSetpoint = "Amp";
    }

    public void intakeNote(){
//DO OTHER PLEASe
    if((encoder2.getDistance() >= WristConstants.speakerScoreSetpoint + WristConstants.kArmOffsetRads - 2) && (encoder2.getDistance() <= WristConstants.speakerScoreSetpoint + WristConstants.kArmOffsetRads + 2)){
      setGoal(ShoulderConstants.speakerScoreSetpoint);
      currentSetpoint = "intakeNoteTransitionNear";
    }else if((encoder2.getDistance() >= WristConstants.farSpeakerScoreSetpoint + WristConstants.kArmOffsetRads - 2) && (encoder2.getDistance() <= WristConstants.farSpeakerScoreSetpoint + WristConstants.kArmOffsetRads + 2)){
      setGoal(ShoulderConstants.speakerScoreSetpoint);
      currentSetpoint = "intakeNoteTransitionFar"; 
    }else{
    setGoal(WristConstants.zeroSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "intakeNote";
  }


     setGoal(ShoulderConstants.intakeNoteSetpoint);
     currentSetpoint = "Ground";
    }

    public void zero(){
      setGoal(ShoulderConstants.zeroSetpoint);
     currentSetpoint = "Zero";
    }

    public void speakerScore(){
      setGoal(ShoulderConstants.speakerScoreSetpoint); //check shuffleboard for real values
      currentSetpoint = "Speaker";

    }

    // public void displaySetpoint(){
    //   SmartDashboard.putNumber("Shoulder Setpoint", currentSetpoint);
    
    // }

}
