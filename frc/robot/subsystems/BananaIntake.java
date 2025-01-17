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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 

public class BananaIntake extends SubsystemBase {
  Double leftTrigger;
  Double rightTrigger;
  // String Pose = WristSubsystem.currentSetpoint;
  
  private final TalonFX bottomShooter;
  private final TalonFX topIntake;
  private final TalonFX topShooter;
  private final TalonFX bottomIntake;
 private boolean NoteSeen = false;

  public DutyCycleOut bottomIntakeMP;
  public  DutyCycleOut topShooterMP;
  public  DutyCycleOut topIntakeMP;
  public  DutyCycleOut bottomShooterMP;

  //Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.COLORSENS_PORT);
  public String ColorString1;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  //private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kNoteTarget = new Color(0.573, 0.355, 0.072);
  
  public BananaIntake() {
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

  // Color Sensor
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
   // m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kNoteTarget);  
  
  
  }

  public void bananaFwdShooter(){
  SmartDashboard.putNumber("DutyCycle1", bottomIntakeMP.Output);
  
  
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
  public void bananaFwdIntake(){
  Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      ColorString1 = "Blue";
    } else if (match.color == kGreenTarget) {
      ColorString1 = "Green";
    } else if (match.color == kNoteTarget) {
      ColorString1 = "Note";
    } else {
      ColorString1 = "Unknown";
    }
    SmartDashboard.putString("ColorString1", ColorString1);
    //SmartDashboard.putString("Pose", Pose);
    // This method will be called once per scheduler run
  if (ColorString1 != "Note" & NoteSeen == false ){
  bottomIntakeMP.Output = 0; // -.5
  topIntakeMP.Output = 0;     // .5
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  }else if (NoteSeen = true & ColorString1 != "Note"){
  bottomIntakeMP.Output = .05;
  topIntakeMP.Output = -.05;
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output).withOverrideBrakeDurNeutral(true);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output).withOverrideBrakeDurNeutral(true);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  }else{
  bottomIntakeMP.Output = 0;
  topIntakeMP.Output = 0;
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output).withOverrideBrakeDurNeutral(true);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output).withOverrideBrakeDurNeutral(true);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  NoteSeen = true;

  }

  
  }
  public void bananaFwdIntakeOverRide(){
  SmartDashboard.putNumber("DutyCycle1", bottomIntakeMP.Output);
  bottomIntakeMP.Output = -1;
  topIntakeMP.Output = -1;
  // bottomShooterMP.Output = -1;
  // topShooterMP.Output = 1;
  bottomShooterMP = new DutyCycleOut(bottomIntakeMP.Output);
  // topShooterMP = new DutyCycleOut(topShooterMP.Output);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
  // bottomShooterMP = new DutyCycleOut(bottomShooterMP.Output);
  // bottomShooter.setControl(bottomShooterMP);
  // topShooter.setControl(topShooterMP);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  NoteSeen = false;
  }
  public void bananaBwd(){

  bottomIntakeMP.Output = .1;
  topIntakeMP.Output = .1;
  //bottomShooterMP.Output = 1;
  bottomShooterMP = new DutyCycleOut(bottomIntakeMP.Output);
  // topShooterMP = new DutyCycleOut(topShooterMP.Output);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);

  //topShooterMP.Output = -1;
    bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  }
  public void doNothingShooter(){
   bottomIntakeMP.Output = 0;
  topShooterMP.Output = 0;
  // bottomIntakeMP.Output = 0;
  // topIntakeMP.Output = 0;
  // bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  topShooterMP = new DutyCycleOut(topShooterMP.Output);
  // topIntakeMP = new DutyCycleOut(bottomShooterMP.Output);
  bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
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
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
  // bottomShooterMP = new DutyCycleOut(topShooterMP.Output);
  // bottomShooter.setControl(bottomShooterMP);
  // topShooter.setControl(topShooterMP);
  bottomIntake.setControl(bottomIntakeMP);
  topIntake.setControl(topIntakeMP);
  
  }

  public void doNothingEject(){
  //  bottomShooterMP.Output = 0;
  // topShooterMP.Output = 0;
 bottomIntakeMP.Output = 0;
  topIntakeMP.Output = 0;
 bottomIntakeMP = new DutyCycleOut(bottomIntakeMP.Output);
  // topShooterMP = new DutyCycleOut(topShooterMP.Output);
  topIntakeMP = new DutyCycleOut(topIntakeMP.Output);
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


