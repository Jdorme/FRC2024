// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/** Add your docs here. */
public class Constants {
   //ids
   public static final int bottomShooterID = 20;
   public static final int topIntakeID = 21;
   public static final int topShooterID = 22;
   public static final int bottomIntakeID = 23;
   public static final I2C.Port COLORSENS_PORT = I2C.Port.kOnboard;
   
   public static final int liftID = 38;

   public static final class ShoulderConstants {
      public static final int shoulderMotorID = 34;
  
      public static final double kP = 1;
  
      // These are fake gains; in actuality these must be determined individually for each robot
      public static final double kSVolts = 1;
      public static final double kGVolts = 1;
      public static final double kVVoltSecondPerRad = 2;
      public static final double kAVoltSecondSquaredPerRad = .25;
  
      public static final double kMaxVelocityRadPerSecond = 100;
      public static final double kMaxAccelerationRadPerSecSquared = 80;
  
      //public static final int shoulderEncoderPort = 0;
      public static final int kEncoderPPR = 256;
      public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
  
      public static final double distancePerRotation =1; 
      // was 0.0992758492;
      // The offset of the arm from the horizontal in its neutral position,
      // measured from the horizontal
      public static final double kArmOffsetRads = 0;

      public static final double ampAngle = .1;

      public static final double speakerScoreSetpoint = 0; //15
      public static final double farSpeakerScoreSetpoint = 0; //14.149
      public static final double ampScoreSetpoint = 0;
      public static final double intakeNoteSetpoint = 52.579;
      public static final double zeroSetpoint = 0;

    }


   public static final class WristConstants {
      public static final int wristMotorID = 35;
  
      // These are fake gains; in actuality these must be determined individually for each robot
      public static final double kSVolts = 1;
      public static final double kGVolts = 1;
      public static final double kVVoltSecondPerRad = 2;
      public static final double kAVoltSecondSquaredPerRad = .25;
  
      public static final double kMaxVelocityRadPerSecond = 350;
      public static final double kMaxAccelerationRadPerSecSquared = 275;
  
      //public static final int shoulderEncoderPort = 0;
      public static final int kEncoderPPR = 256;
      public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
  
      public static final double distancePerRotation =1; 
      // was 0.0992758492;
      // The offset of the arm from the horizontal in its neutral position,
      // measured from the horizontal
      //public static final double kArmOffsetRads = 44;
      public static final double kArmOffsetRads = 0;
      public static final double ampAngle = .1;


     public static final double speakerScoreSetpoint = -18;
      public static final double farSpeakerScoreSetpoint = -9;
      public static final double ampScoreSetpoint = -67;
      public static final double intakeNoteSetpoint = -32;
      public static final double zeroSetpoint = 0;
     
      // public static final double speakerScoreSetpoint = 35.636;
      // public static final double farSpeakerScoreSetpoint = 37;
      // public static final double ampScoreSetpoint = -18.593;
      // public static final double intakeNoteSetpoint = 11.682;
      // public static final double zeroSetpoint = -43.9877; //this bring it to what the old zero was -
    }

}
