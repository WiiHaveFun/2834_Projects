// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Drive Motor CAN IDs
    public static final int kFrontRightDriveMotorID = 1;
    public static final int kFrontLeftDriveMotorID = 2;
    public static final int kRearLeftDriveMotorID = 3;
    public static final int kRearRightDriveMotorID = 4;

    // Turning Motor CAN IDs
    public static final int kFrontRightSteerMotorID = 5;
    public static final int kFrontLeftSteerMotorID = 6;
    public static final int kRearLeftSteerMotorID = 7;
    public static final int kRearRightSteerMotorID = 8;

    // Drive Motor Inversion
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    // Steering Motor Inversion
    public static final boolean kFrontRightSteerMotorReversed = false;
    public static final boolean kFrontLeftSteerMotorReversed = false;
    public static final boolean kRearLeftSteerMotorReversed = false;
    public static final boolean kRearRightSteerMotorReversed = false;

    // Turning Encoder Sensor Phase
    public static final boolean kFrontRightSteerEncoderReversed = true;
    public static final boolean kFrontLeftSteerEncoderReversed = true;
    public static final boolean kRearLeftSteerEncoderReversed = true;
    public static final boolean kRearRightSteerEncoderReversed = true;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.565;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.565;

    // Swerve Drive Kinematics Object
    // Different from WPIlib docs, +X is forward, +Y is right
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                                                                           new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                                                                           new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                                                                                           new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


    public static final int kFrontRightZero = 10035;
    public static final int kFrontLeftZero = 18170;
    public static final int kRearLeftZero = -40151;
    public static final int kRearRightZero = -8765;
  }

  public static final class ModuleConstants {
    // Drive motor distance (m) per motor rotation calculation
    public static final double kDriveGearReduction = 1 / 4.71; // 1 wheel rotation per 4.71 motor rotations
    public static final double kWheelDiameterMeters = 0.0762; // 0.0762 m diameter
    public static final double kDriveEncoderDistancePerMotorRotation = kDriveGearReduction * (kWheelDiameterMeters * Math.PI); // Motor rotations to wheel distance conversion
    public static final double kDriveEncoderMetersPerSecondPerMotorRotationPerMinute = kDriveEncoderDistancePerMotorRotation * (1.0 / 60.0); // Motor rotations per minute to meters per second conversion

    // Steering motor radians per pulse calculation
    public static final int kTurningEncoderCPR = 4096;
    public static final double kTurningEncoderDistancePerPulse = (2.0 * Math.PI) / (double) kTurningEncoderCPR; // Assumes the encoders are on a 1:1 reduction with the module shaft.
        
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    public static final double nominalTurnOutputPercent = 0.95;
    public static final int encoderTickAtNominal = 1245;
      
    public static final double fGain = (nominalTurnOutputPercent * 1023) / encoderTickAtNominal;

    public static final double pGain = 0.2;
    public static final double error = 193;
    public static final double pTerm = (pGain * 1023) / error;
    public static final double dGain = 10*pTerm;

    public static final Gains kGains = new Gains(pTerm, 0.005, dGain, fGain, 120, 0.0);

    // Steering module cruise velocity and acceleration
    public static final int MODULE_CRUISE_VELOCITY = 1245;
    public static final int MODULE_ACCELERATION = 3735;


    // Drive motor velocity pid parameters
    public static final double velocityAtNominal = 4.7; // meters per second

    public static double kP = 0.02;
    public static double kI = 0.0001;
    public static double kD = 1.5;
    public static double kIz = 0.1;
    public static double kFF = 1 / velocityAtNominal;
    public static double kMaxOutput = 1.0;
    public static double kMinOutput = -1.0;
    public static double maxRPM = 4.0; // meters per second
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = (Math.PI / 2) / 8.0;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
