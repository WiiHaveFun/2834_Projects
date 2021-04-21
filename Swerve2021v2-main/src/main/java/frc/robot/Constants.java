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

    // Steering Motor CAN IDs
    public static final int kFrontRightSteerMotorID = 5;
    public static final int kFrontLeftSteerMotorID = 6;
    public static final int kRearLeftSteerMotorID = 7;
    public static final int kRearRightSteerMotorID = 8;

    // Steering Motor CANCoder IDs
    public static final int kFrontRightSteerCANCoderID = 20;
    public static final int kFrontLeftSteerCANCoderID = 21;
    public static final int kRearLeftSteerCANCoderID = 22;
    public static final int kRearRightSteerCANCoderID = 23;

    // Drive Motor Inversion
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = false;

    // Steering Motor Inversion
    public static final boolean kFrontRightSteerMotorReversed = false;
    public static final boolean kFrontLeftSteerMotorReversed = false;
    public static final boolean kRearLeftSteerMotorReversed = false;
    public static final boolean kRearRightSteerMotorReversed = false;

    // Drive Encoder Sensor Phase
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    // Steering Encoder Sensor Phase
    public static final boolean kFrontRightSteerEncoderReversed = false;
    public static final boolean kFrontLeftSteerEncoderReversed = false;
    public static final boolean kRearLeftSteerEncoderReversed = false;
    public static final boolean kRearRightSteerEncoderReversed = false;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.62;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.595;

    // Swerve Drive Kinematics Object
    // Different from WPIlib docs, +X is forward, +Y is right
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                                                                           new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                                                                           new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                                                                                           new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // public static final int kFrontRightZero = -4293;
    // public static final int kFrontLeftZero = -2673;
    // public static final int kRearLeftZero = -15511;
    // public static final int kRearRightZero = -639;
    public static final int kFrontRightZero = 0;
    public static final int kFrontLeftZero = 0;
    public static final int kRearLeftZero = 0;
    public static final int kRearRightZero = 0;
  }

  public static final class ModuleConstants {
    // Drive motor distance (m) per motor rotation calculation
    public static final double kDriveEncoderTicksPerRotation = 2048.0;
    public static final double kDriveGearReduction = 1.0 / 6.0; // 1 wheel rotation per 6 motor rotations
    public static final double kWheelDiameterMeters = 0.076708; // 0.0762 m diameter empirical measurement 3.02 in
    public static final double kDriveEncoderDistancePerTick = (1.0 / kDriveEncoderTicksPerRotation) * kDriveGearReduction * (kWheelDiameterMeters * Math.PI); // Motor rotations to wheel distance conversion
    public static final double kDriveEncoderMetersPerSecondPerEncoderTickPer100Ms = kDriveEncoderDistancePerTick * 10.0; // Motor rotations per minute to meters per second conversion

    // Steering analog encoder radians per pulse calculation
    public static final int kTurningAuxEncoderCPR = 1024;
    public static final double kTurningAuxEncoderDistancePerPulse = (2.0 * Math.PI) / (double) kTurningAuxEncoderCPR; // Assumes the encoders are on a 1:1 reduction with the module shaft.
        
    // Steering CANCoder radains per pusle calculation
    public static final double kCANCoderGearReduction = 49.0 / 12.0;
    public static final double kTurningEncoderCPR = 4096.0 * kCANCoderGearReduction;
    public static final double kTurningEncoderDistancePerPulse = (2.0 * Math.PI) / (double) kTurningEncoderCPR;

    // Drive motor ramp rate in seconds
    public static final double kOpenRampRate = 0.333;

    // Timeout for all talons
    public static final int kTimeoutMs = 30;

    // Drive PID parameters
    public static final int kDriveSlotIdx = 0;
    public static final int kDrivePIDLoopIdx = 0;

    public static final double nominalDriveOutputPercent = 0.95;
    public static final int driveEncoderTickAtNominal = 20322;
      
    public static final double fDriveGain = (nominalDriveOutputPercent * 1023) / driveEncoderTickAtNominal;

    public static final double pDriveGain = 0.001; //0.2
    public static final double errorDrive = 58;
    public static final double pDriveTerm = (pDriveGain * 1023) / errorDrive;
    public static final double dDriveGain = 50*pDriveTerm;

    public static final Gains kDriveGains = new Gains(pDriveTerm, 0.000, dDriveGain, fDriveGain, 50, 0.0);

    // Primary PID parameters
    public static final int kRemoteID = 0;
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;

    public static final double nominalTurnOutputPercent = 0.95;
    public static final int encoderTickAtNominal = 5600;
      
    public static final double fGain = (nominalTurnOutputPercent * 1023) / encoderTickAtNominal;

    public static final double pGain = 0.7; //0.2
    public static final double error = 1190;
    public static final double pTerm = (pGain * 1023) / error;
    public static final double dGain = 10*pTerm;

    public static final Gains kGains = new Gains(pTerm, 0.0001, dGain, fGain, 50, 0.0);

    // Steering module cruise velocity and acceleration
    public static final int MODULE_CRUISE_VELOCITY = 5600;
    public static final int MODULE_ACCELERATION = 5600;

    // Aux PID parameters
    public static final int kAuxSlotIdx = 1;

    public static final double auxNominalTurnOutputPercent = 0.95;
    public static final int auxEncoderTickAtNominal = 325;
      
    public static final double fGainAux = (auxNominalTurnOutputPercent * 1023) / auxEncoderTickAtNominal;

    public static final double pGainAux = 0.5; //0.2
    public static final double errorAux = 135;
    public static final double pTermAux = (pGainAux * 1023) / errorAux;
    public static final double dGainAux = 0*pTermAux;

    public static final Gains kAuxGains = new Gains(pTermAux, 0.0000, dGainAux, fGainAux, 50, 0.0);
    // public static final Gains kGains = new Gains(pTerm, 0.0, dGain, fGain, 0, 0.0);

    // Steering module cruise velocity and acceleration
    public static final int AUX_MODULE_CRUISE_VELOCITY = 325;
    public static final int AUX_MODULE_ACCELERATION = 325;
  }

  public static final class OIConstants {
    // Controller(s)
    public static final int kDriverControllerPort = 0;
    // Buttons
    public static final int kIntakeButtonID = 6;
    public static final int kEjectButtonID = 1;
    public static final int kFeedButtonID = 5;
    public static final int kToggleIntakeButtonID = 2;
    public static final int kEnableAutoAimButtonID = 3;
    public static final int kEnableAutoTrackButtonID = 1;
    public static final int kReverseBrush = 4;
    public static final int kForwardABit = 7;
    public static final int kBackwardABit = 8;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.9;
    public static final double kMaxAccelerationMetersPerSecondSquared = 11.7;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = (Math.PI / 2) / 8.0;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class FeederConstants {
    public static final int kFeederMotorID = 16;
    public static final boolean kFeederMotorReversed = false;

    public static final double kNeutralPower = 0.0;
    public static final double kFeedPower = 0.6;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorID = 17;
    public static final boolean kIntakeMotorReversed = true;

    public static final double kNeutralPower = 0.0;
    public static final double kIntakePower = 0.75;
    public static final double kEjectPower = 0.0;
  }

  public static final class BrushConstants {
    public static final int kBrushMotorID = 18;
    public static final boolean kBrushMotorReversed = true;

    public static final double kNeutralPower = 0.0;
    public static final double kNominalPower = 0.4;
  }

  public static final class PneumaticsConstants {
    public static final int kPCMID = 30;
    public static final int kSolenoidChannel = 0;
  }

  public static final class ShooterConstants {
    public static final int kFlyLeftID = 15;
    public static final int kFlyRightID = 16;
    public static final int kTurretID = 29;

    public static final int kLeftServoID = 1;
    public static final int kRightServoID = 0;

    public static final boolean kFlyLeftReversed = true;
    public static final boolean kFlyRightReversed = false;
    public static final boolean kTurretReversed = false;

    public static final boolean kFlyEncoderReversed = false;

    // Timeout for all talons
    public static final int kTimeoutMs = 30;

    // Shooter Encoder Conversions
    public static final double RPMtoTicksPer100ms = (1.0 / 60.0) * (1.0 / 10.0) * 2048.0;

    // Flywheel closed loop ramp rate
    public static final double kClosedLoopRampRate = 1.0;

    // Flywheel PID parameters
    public static final int kFlySlotIdx = 0;
    public static final int kFlyPIDLoopIdx = 0;

    public static final double nominalFlyOutputPercent = 0.95;
    public static final int flyEncoderTickAtNominal = 18443;
      
    public static final double fFlyGain = (nominalFlyOutputPercent * 1023) / flyEncoderTickAtNominal;

    public static final double pFlyGain = 0.11; //0.2
    public static final double errorFly = 123;
    public static final double pFlyTerm = (pFlyGain * 1023) / errorFly;
    public static final double dFlyGain = 10*pFlyTerm;

    public static final Gains kFlyGains = new Gains(pFlyTerm, 0.000, dFlyGain, fFlyGain, 50, 0.0);

    // Turret Encoder Conversions
    public static final double turretGearRatio = (1.0 / 25.0) * (18.0 / 202.0);
    public static final double turretDegreesPerMotorRot = turretGearRatio * 360.0;
    public static final double turretDegPerSecPerMotorRMP = turretDegreesPerMotorRot / 60.0;

    // Turret PID parameters
    public static final int kTurretSlotIdx = 0;

    public static final double nominalTurretOutputPercent = 0.2;
    public static final int velocityAtNominal = 46;
      
    public static final double fGain = nominalTurretOutputPercent / velocityAtNominal;

    public static final double pGain = 0.01; //0.2
    public static final double error = 2;
    public static final double pTerm = pGain / error;
    public static final double dGain = 10*pTerm;

    public static final Gains kGains = new Gains(pTerm, 0.0, dGain, fGain, 0, 0.5);

    // Turret cruise velocity and acceleration
    public static final int MODULE_CRUISE_VELOCITY = 180;
    public static final int MODULE_ACCELERATION = 180;

    public static final double kAllowedError = 0.1;

    // Hood PID parameters
    public static final double kPHood = 0.01;
    public static final double kIHood = 0.0;
    public static final double kDHood = 0.0;
  }
}
