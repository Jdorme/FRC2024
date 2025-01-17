// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  
  private final TalonFX m_motor;

    public DutyCycleOut m_motorMP;
  

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {

m_motor = new TalonFX(Constants.liftID, "rio");

m_motorMP = new DutyCycleOut(0);

m_motor.setControl(m_motorMP);
  }

  //This Make lift go up and down
  public void liftUp() {
    m_motorMP.Output = 1;//May need to change, depending on if not up or down//
    m_motorMP = new DutyCycleOut(m_motorMP.Output).withOverrideBrakeDurNeutral(true);;
    m_motor.setControl(m_motorMP);

  }

    public void liftDown() {
     m_motorMP.Output = -1;//May need to change, depending on if not up or down//
    m_motorMP = new DutyCycleOut(m_motorMP.Output).withOverrideBrakeDurNeutral(true);;
    m_motor.setControl(m_motorMP);
  }
  public void liftHold(){

    m_motorMP.Output = 0;
    m_motorMP = new DutyCycleOut(m_motorMP.Output).withOverrideBrakeDurNeutral(true);;
    m_motor.setControl(m_motorMP);
    // m_motor.setNeutralMode(NeutralModeValue.Brake);
    
    //.withOverrideBrakeDurNeutral(true);
  }


}
