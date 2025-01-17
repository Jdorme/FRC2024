// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 

public class Shooter extends SubsystemBase {
  Double leftTrigger;
  Double rightTrigger;
  
  private final TalonFX bottomShooter;
  private final TalonFX topIntake;
  private final TalonFX topShooter;
  private final TalonFX bottomIntake;


  public DutyCycleOut bottomIntakeMP;
  public  DutyCycleOut topShooterMP;
  public  DutyCycleOut topIntakeMP;
  public  DutyCycleOut bottomShooterMP;
  
  public Shooter() {
  bottomShooter = new TalonFX(Constants.bottomShooterID, "rio");
  topIntake = new TalonFX(Constants.topIntakeID, "rio");
  topShooter = new TalonFX(Constants.topShooterID, "rio");
  bottomIntake = new TalonFX(Constants.bottomIntakeID, "rio");
  // SmartDashboard.putNumber("Left Trigger Axis", rightTrigger1);
  
  bottomIntakeMP = new DutyCycleOut(0);
  topShooterMP = new DutyCycleOut(0);
  topIntakeMP = new DutyCycleOut(0);
  bottomShooterMP = new DutyCycleOut(0);
  SmartDashboard.putNumber("DutyCycle", bottomIntakeMP.Output);

  bottomShooter.setControl(bottomShooterMP);
  topShooter.setControl(topShooterMP);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  }

  @Override
  public  void periodic() {
    // This method will be called once per scheduler run
  }
  public void bananaFwdShooter(){
  SmartDashboard.putNumber("DutyCycle1", bottomIntakeMP.Output);
  // bottomIntakeMP.Output = -1;
  // topIntakeMP.Output = 1;
  bottomShooterMP.Output = 1;
  topShooterMP.Output = -1;
  // bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  topShooterMP = new DutyCycleOut(topShooterMP.Output);
  // topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
  bottomShooterMP = new DutyCycleOut(bottomShooterMP.Output);
  bottomShooter.setControl(bottomShooterMP);
  topShooter.setControl(topShooterMP);
  // bottomIntake.setControl(bottomIntakeMP);
  // topIntake.setControl(topIntakeMP);
  }
  public void bananaFwdIntake(){
  SmartDashboard.putNumber("DutyCycle1", bottomIntakeMP.Output);
  bottomIntakeMP.Output = -.25;
  topIntakeMP.Output = .25;
  // bottomShooterMP.Output = -1;
  // topShooterMP.Output = 1;
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  // topShooterMP = new DutyCycleOut(topShooterMP.Output);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
  // bottomShooterMP = new DutyCycleOut(bottomShooterMP.Output);
  // bottomShooter.setControl(bottomShooterMP);
  // topShooter.setControl(topShooterMP);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  }
  public void bananaBwd(){
    // bottomIntakeMP.Output = -1;
    // topIntakeMP.Output = 1;
    bottomShooterMP.Output = -1;
    topShooterMP.Output = 1;
    // bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
    topShooterMP = new DutyCycleOut(topShooterMP.Output);
    // topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
    bottomShooterMP = new DutyCycleOut(bottomShooterMP.Output);
    bottomShooter.setControl(bottomShooterMP);
    topShooter.setControl(topShooterMP);
    // bottomIntake.setControl(bottomIntakeMP);
    // topIntake.setControl(topIntakeMP);
  }
  public void doNothingShooter(){
   bottomShooterMP.Output = 0;
  topShooterMP.Output = 0;
  // bottomIntakeMP.Output = 0;
  // topIntakeMP.Output = 0;
  // bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  topShooterMP = new DutyCycleOut(topShooterMP.Output).withOverrideBrakeDurNeutral(true);
  // topIntakeMP = new DutyCycleOut(bottomShooterMP.Output);
  bottomShooterMP = new DutyCycleOut(topShooterMP.Output).withOverrideBrakeDurNeutral(true);
  bottomShooter.setControl(bottomShooterMP);
  topShooter.setControl(topShooterMP);
  // bottomIntake.setControl(bottomIntakeMP);
  // topIntake.setControl(topIntakeMP);
  
  }
  public void doNothingIntake(){
  //  bottomShooterMP.Output = 0;
  // topShooterMP.Output = 0;
  bottomIntakeMP.Output = 0;
  topIntakeMP.Output = 0;
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  // topShooterMP = new DutyCycleOut(topShooterMP.Output);
  topIntakeMP = new DutyCycleOut(bottomShooterMP.Output);
  // bottomShooterMP = new DutyCycleOut(topShooterMP.Output);
  // bottomShooter.setControl(bottomShooterMP);
  // topShooter.setControl(topShooterMP);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  
  }


}

//idk if the names of the double supplier are still valid
        /* Get forward and rotational throttle from joystick */
    /* invert the joystick Y because forward Y is negative */
        // double leftTrigger = joystick.getLeftTriggerAxis();
        // double rightTrigger = joystick.getRightTriggerAxis();
    /* Set output to control frames */
  



  //   if (bananaDirection==true){
  //   // bottomShooterMP.Output = -rightTrigger.getAsDouble();
  //   // topShooterMP.Output = rightTrigger.getAsDouble();
  //   // bottomIntakeMP.Output = -leftTrigger.getAsDouble();
  //   // topIntakeMP.Output = leftTrigger.getAsDouble();
  //   }else if (bananaDirection==false){
  //   bottomShooterMP.Output = rightTrigger.getAsDouble();
  //   topShooterMP.Output = -rightTrigger.getAsDouble();
  //   bottomIntakeMP.Output = leftTrigger.getAsDouble();
  //   topIntakeMP.Output = -leftTrigger.getAsDouble();
  //   }
    

  //   if(buttonB.getAsBoolean()){
  //   bananaDirection = true;
  //   }

  //   if(buttonA.getAsBoolean()){
  //   bananaDirection = false;
  //   }

  //    bottomShooter.setControl(bottomShooterMP);
  //     topShooter.setControl(topShooterMP);
  //     bottomIntake.setControl(bottomIntakeMP);
  //     topIntake.setControl(topIntakeMP);
  // }

  // public void bananaIntakeSpinForward(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger){
  //   bottomShooterMP.Output = -rightTrigger.getAsDouble();
  //   topShooterMP.Output = rightTrigger.getAsDouble();
  //   bottomIntakeMP.Output = -leftTrigger.getAsDouble();
  //   topIntakeMP.Output = leftTrigger.getAsDouble();
  // }

  // public void bananaIntakeSpinReverse(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger){
  //   bottomShooterMP.Output = rightTrigger.getAsDouble();
  //   topShooterMP.Output = -rightTrigger.getAsDouble();
  //   bottomIntakeMP.Output = leftTrigger.getAsDouble();
  //   topIntakeMP.Output = -leftTrigger.getAsDouble();
 // }


