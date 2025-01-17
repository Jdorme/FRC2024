// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LemonLight extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    NetworkTableEntry tid;
    double aprilTag;
    double Kp = 0.02; // Proportional control constant
    double min_command = 0.15; // Minimum amount to slightly move
    double steering_adjust;


    public LemonLight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        table.getEntry("pipeline").setNumber(0);


    }



    public double getXPos(){
        return tx.getDouble(0.0);
    }

    public double getTV(){
        return tv.getDouble(0.0);
    }

    public double getYPos(){
        return ty.getDouble(0.0);
    }

    public double getArea(){
       return ta.getDouble(0.0);
    }

    public double getID(){
        return tid.getDouble(0);
    }

    public double getRotationAdjust() {
        double tx = table.getEntry("tx").getDouble(0);
        double heading_error = -tx;
        if (tx > 1.0) {
          steering_adjust = Kp * heading_error - min_command;
        } else if (tx < 1.0) {
          steering_adjust = Kp * heading_error + min_command;
        }
        return steering_adjust;
      }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("XPos", getXPos());
        SmartDashboard.putNumber("YPos", getYPos());
        SmartDashboard.putNumber("Area", getArea());
        SmartDashboard.putNumber("April Tag", getID());
    }
         // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= 1.5 * Math.PI;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= 6.4;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;

    }

    
 
}
