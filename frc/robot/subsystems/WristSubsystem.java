// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.subsystems.ShoulderArmSubsystem;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** A robot arm subsystem that moves with a motion profile. */
public class WristSubsystem extends ProfiledPIDSubsystem {

  public static String currentSetpoint;

  private final TalonFX m_motor = new TalonFX(35, "rio");
  static Encoder encoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
       WristConstants.kSVolts, WristConstants.kGVolts,
       WristConstants.kVVoltSecondPerRad, WristConstants.kAVoltSecondSquaredPerRad);
Encoder encoder2 = ShoulderArmSubsystem.encoder;
  /** Create a new ArmSubsystem. */
  public WristSubsystem() {
    super(
        new ProfiledPIDController(
            .25,
            0,
            0,
            new TrapezoidProfile.Constraints(
             WristConstants.kMaxVelocityRadPerSecond,
             WristConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    encoder.setDistancePerPulse(WristConstants.kEncoderDistancePerPulse);
    
    // Start arm at rest in neutral position
    setGoal(WristConstants.kArmOffsetRads);
    

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
    m_motor.setVoltage((output));
    //SmartDashboard.putNumber("Offset Wrist Encoder", encoder.getDistance() + WristConstants.kArmOffsetRads);
    SmartDashboard.putNumber("Wrist Encoder", encoder.getDistance());
    SmartDashboard.putNumber("2nd shoulder encoder", encoder2.getDistance());
    SmartDashboard.putString("Wrist Setpoint", currentSetpoint);
    
  }
@Override
public void periodic() {
    if(encoder2.getDistance() < 35){
    
      if(currentSetpoint == "SpeakerScoreTransition"){
        setGoal(WristConstants.farSpeakerScoreSetpoint + WristConstants.kArmOffsetRads);
      }else if(currentSetpoint == "ZeroTransition"){
        setGoal(WristConstants.zeroSetpoint + WristConstants.kArmOffsetRads);
      }else if(currentSetpoint == "FarSpeakerScoreTransition"){
        setGoal(WristConstants.farSpeakerScoreSetpoint + WristConstants.kArmOffsetRads);
      }

    }

    // if(currentSetpoint == "GroundTransition"){
    //   if(encoder.getDistance() <= -15 || encoder.getDistance() >= -21){

    // setGoal(WristConstants.intakeNoteSetpoint + WristConstants.kArmOffsetRads);
    // currentSetpoint = "Ground";

    //   }


    // }
    // if((encoder.getDistance() <= -15 || encoder.getDistance() >= -21)){

    // }


    super.periodic();
}
  @Override
  public double getMeasurement() {
    return encoder.getDistance() + WristConstants.kArmOffsetRads;
  }

  public void ampScore(){
    // while(encoder2.getDistance() < 25){
    //   setGoal(-20);
     //}
    setGoal(WristConstants.ampScoreSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "Amp";
   }

   public void intakeNote(){

    // if((encoder.getDistance() >= -16 && encoder.getDistance() <= -20) && currentSetpoint == "SpeakerScoreTransition"){
    //   setGoal(0 + WristConstants.kArmOffsetRads); //idk
    //   currentSetpoint = "GroundTransition";
    // }else{
    setGoal(WristConstants.intakeNoteSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "Ground";
  // }

   }

   public void zero(){
    if(encoder2.getDistance() > 25){
      setGoal(-32);
      currentSetpoint = "ZeroTransition";
    }else{
    setGoal(WristConstants.zeroSetpoint + WristConstants.kArmOffsetRads);
    currentSetpoint = "Zero";
  }
  }

  public void speakerScore(){
    if(encoder2.getDistance() > 25){
      setGoal(-32);
      currentSetpoint = "SpeakerScoreTransition";
    }else{
    setGoal(WristConstants.speakerScoreSetpoint + WristConstants.kArmOffsetRads); //check shuffleboard!!!!!!!!!!!!!!!!!!!!!!!!!
    currentSetpoint = "Speaker";
  }
  }

    public void farSpeakerScore(){
    if(encoder2.getDistance() > 25){
      setGoal(-32);
      currentSetpoint = "SpeakerScoreTransition";
    }else{
    setGoal(WristConstants.farSpeakerScoreSetpoint + WristConstants.kArmOffsetRads); //check shuffleboard!!!!!!!!!!!!!!!!!!!!!!!!!
    currentSetpoint = "FarSpeaker";
  }
  }

  public void inBetween(){
      setGoal(-32);
      currentSetpoint = "Transition";
  }

  // public void displaySetpoint(){
  //   SmartDashboard.putNumber("Wrist Setpoint", currentSetpoint);
  
  // }


}
