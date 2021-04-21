// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Swerve Modules
  private final SwerveModule m_fr;
  private final SwerveModule m_fl;
  private final SwerveModule m_rl;
  private final SwerveModule m_rr;

  // NavX MXP Gyro
  private final AHRS m_gyro;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_fr = new SwerveModule(DriveConstants.kFrontRightDriveMotorID, 
                            DriveConstants.kFrontRightSteerMotorID, 
                            DriveConstants.kFrontRightDriveMotorReversed, 
                            DriveConstants.kFrontRightSteerMotorReversed, 
                            DriveConstants.kFrontRightSteerEncoderReversed, 
                            DriveConstants.kFrontRightZero);

    m_fl = new SwerveModule(DriveConstants.kFrontLeftDriveMotorID, 
                            DriveConstants.kFrontLeftSteerMotorID, 
                            DriveConstants.kFrontLeftDriveMotorReversed, 
                            DriveConstants.kFrontLeftSteerMotorReversed, 
                            DriveConstants.kFrontLeftSteerEncoderReversed, 
                            DriveConstants.kFrontLeftZero);

    m_rl = new SwerveModule(DriveConstants.kRearLeftDriveMotorID, 
                            DriveConstants.kRearLeftSteerMotorID, 
                            DriveConstants.kRearLeftDriveMotorReversed, 
                            DriveConstants.kRearLeftSteerMotorReversed, 
                            DriveConstants.kRearLeftSteerEncoderReversed, 
                            DriveConstants.kRearLeftZero);

    m_rr = new SwerveModule(DriveConstants.kRearRightDriveMotorID, 
                            DriveConstants.kRearRightSteerMotorID, 
                            DriveConstants.kRearRightDriveMotorReversed, 
                            DriveConstants.kRearRightSteerMotorReversed, 
                            DriveConstants.kRearRightSteerEncoderReversed, 
                            DriveConstants.kRearRightZero);

    m_gyro = new AHRS(Port.kMXP);
    m_gyro.zeroYaw();

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), new Pose2d(0, 0, Rotation2d.fromDegrees(m_gyro.getYaw())));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getYaw());

    m_odometry.update(gyroAngle, m_fr.getState(), m_fl.getState(), m_rl.getState(), m_rr.getState());

    SmartDashboard.putNumber("Gyro Yaw", m_gyro.getYaw());

    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
    
    SmartDashboard.putNumber("FR_Drive_Distance", m_fr.getDrivePosition());
    SmartDashboard.putNumber("FL_Drive_Distance", m_fl.getDrivePosition());
    SmartDashboard.putNumber("RL_Drive_Distance", m_rl.getDrivePosition());
    SmartDashboard.putNumber("RR_Drive_Distance", m_rr.getDrivePosition());

    SmartDashboard.putNumber("FR_Drive_Velocity", m_fr.getDriveVelocity());
    SmartDashboard.putNumber("FL_Drive_Velocity", m_fl.getDriveVelocity());
    SmartDashboard.putNumber("RL_Drive_Velocity", m_rl.getDriveVelocity());
    SmartDashboard.putNumber("RR_Drive_Velocity", m_rr.getDriveVelocity());

    SmartDashboard.putNumber("FR_Angle", m_fr.getTurningMotorEncoderRadians());
    SmartDashboard.putNumber("FL_Angle", m_fl.getTurningMotorEncoderRadians());
    SmartDashboard.putNumber("RL_Angle", m_rl.getTurningMotorEncoderRadians());
    SmartDashboard.putNumber("RR_Angle", m_rr.getTurningMotorEncoderRadians());
  }

  /**
   * Calculates module angles and speed
   * 
   * @param FWD Joystick y input
   * @param STR Joystick x input
   * @param RCW Joystick 2 x input
   * @param gyroAngle Gyro angle used for field-centric swerve drive
   * @param baseLength Length of wheelbase
   * @param baseWidth Width of wheelbase
   * @return Returns a 2d array containing the speed and angle in degrees for each module
   */
  public static double[][] calculate(double FWD, double STR, double RCW, double gyroAngle, double baseLength, double baseWidth) {
    // Maks the command field-centric
    double temp = FWD * Math.cos(Math.toRadians(gyroAngle)) + STR * Math.sin(Math.toRadians(gyroAngle));
    STR = -FWD * Math.sin(Math.toRadians(gyroAngle)) + STR * Math.cos(Math.toRadians(gyroAngle));
    FWD = temp;

    double R = Math.sqrt(Math.pow(baseLength, 2) + Math.pow(baseWidth, 2));

    double A = STR - RCW * (baseLength / R);
    double B = STR + RCW * (baseLength / R);
    double C = FWD - RCW * (baseWidth / R);
    double D = FWD + RCW * (baseWidth / R);

    // Wheel Speeds
    double fr_ws = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double fl_ws = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double bl_ws = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double br_ws = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

    // Wheel Angles measured clockwise, with zero being straight, from -180 to +180 degrees
    double fr_wa = Math.atan2(B, C) * 180 / Math.PI;
    double fl_wa = Math.atan2(B, D) * 180 / Math.PI;
    double bl_wa = Math.atan2(A, D) * 180 / Math.PI;
    double br_wa = Math.atan2(A, C) * 180 / Math.PI;

    // Normalize wheel speeds
    double max = fr_ws;

    if(fl_ws > max) {
      max = fl_ws;
    } else if(bl_ws > max) {
      max = bl_ws;
    } else if(br_ws > max) {
      max = br_ws;
    }

    if(max > 1) {
      fr_ws /= max;
      fl_ws /= max;
      bl_ws /= max;
      br_ws /= max;
    }

    double[][] output = new double[][] {
      {fr_ws, fr_wa}, 
      {fl_ws, fl_wa},
      {bl_ws, bl_wa}, 
      {br_ws, br_wa}
    };

    return output;
  }

  public void setModuleStates(double[][] desiredStates) {
    m_fr.setState(desiredStates[0][0], desiredStates[0][1], false);
    m_fl.setState(desiredStates[1][0], desiredStates[1][1], false);
    m_rl.setState(desiredStates[2][0], desiredStates[2][1], false);
    m_rr.setState(desiredStates[3][0], desiredStates[3][1], false);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_fr.setState(desiredStates[0].speedMetersPerSecond, desiredStates[0].angle.getDegrees(), true);
    m_fl.setState(desiredStates[1].speedMetersPerSecond, desiredStates[1].angle.getDegrees(), true);
    m_rl.setState(desiredStates[2].speedMetersPerSecond, desiredStates[2].angle.getDegrees(), true);
    m_rr.setState(desiredStates[3].speedMetersPerSecond, desiredStates[3].angle.getDegrees(), true);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw();
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the pose of the robot.
   * 
   * @return The robot's pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}
