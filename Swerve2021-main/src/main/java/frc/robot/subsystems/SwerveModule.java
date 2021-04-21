// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  // Drive Motor
  CANSparkMax m_driveMotor;
  // Steering Motor
  TalonSRX m_steerMotor;

  // Drive Motor Encoder
  CANEncoder m_driveEncoder;

  // Drive Motor PID
  CANPIDController m_pidController;

  // Module Zero
  int m_moduleZero;

  /** 
   * Creates a new SwerveModule. 
   * 
   * @param driveMotorID ID for the drive motor.
   * @param steerMotorID ID for the steering motor.
   * @param driveMotorReversed Direction of the drive motor.
   * @param steerMotorReversed Direction of the steering motor.
   * @param steerEncoderReversed Phase of the steering encoder.
   * @param moduleZero The aboslute encoder count when the module is facing forward.
   * */
  public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed, boolean steerEncoderReversed, int moduleZero) {
    // Motor Init
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_steerMotor = new TalonSRX(steerMotorID);

    // Steer Motor Reset
    m_steerMotor.configFactoryDefault();

    // Encoder Init
    m_driveEncoder = m_driveMotor.getEncoder();

    // Drive Encoder Scaling
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerMotorRotation);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderMetersPerSecondPerMotorRotationPerMinute);

    // Motor Inversions
    m_driveMotor.setInverted(driveMotorReversed);
    m_steerMotor.setInverted(steerMotorReversed);

    // Motor PID Controller Init
    m_pidController = m_driveMotor.getPIDController();

    // PID Parameters
    m_pidController.setP(ModuleConstants.kP);
    m_pidController.setI(ModuleConstants.kI);
    m_pidController.setD(ModuleConstants.kD);
    m_pidController.setIZone(ModuleConstants.kIz);
    m_pidController.setFF(ModuleConstants.kFF);
    m_pidController.setOutputRange(ModuleConstants.kMinOutput, ModuleConstants.kMaxOutput);

    // Select absolute encoder to use
    m_steerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, ModuleConstants.kPIDLoopIdx, ModuleConstants.kTimeoutMs);

    // Steering Encoder Sensor Phase
    m_steerMotor.setSensorPhase(steerEncoderReversed);

    // Set relevant frame periods to be at least as fast as periodic rate
		m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ModuleConstants.kTimeoutMs);
		m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ModuleConstants.kTimeoutMs);

		// Set the peak and nominal outputs
		m_steerMotor.configNominalOutputForward(0, ModuleConstants.kTimeoutMs);
		m_steerMotor.configNominalOutputReverse(0, ModuleConstants.kTimeoutMs);
		m_steerMotor.configPeakOutputForward(1, ModuleConstants.kTimeoutMs);
		m_steerMotor.configPeakOutputReverse(-1, ModuleConstants.kTimeoutMs);

		// Set Motion Magic gains in slot0
		m_steerMotor.selectProfileSlot(ModuleConstants.kSlotIdx, ModuleConstants.kPIDLoopIdx);
		m_steerMotor.config_kF(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kF, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kP(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kP, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kI(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kI, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_kD(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kD, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_IntegralZone(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kIzone, ModuleConstants.kTimeoutMs);

		// Set acceleration and vcruise velocity
		m_steerMotor.configMotionCruiseVelocity(ModuleConstants.MODULE_CRUISE_VELOCITY, ModuleConstants.kTimeoutMs);
    m_steerMotor.configMotionAcceleration(ModuleConstants.MODULE_ACCELERATION, ModuleConstants.kTimeoutMs);

    m_moduleZero = moduleZero;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDrivePosition() { return m_driveEncoder.getPosition(); }

  public double getDriveVelocity() { return m_driveEncoder.getVelocity(); }

  public double getTurningMotorEncoderRadians() { return (m_steerMotor.getSelectedSensorPosition() - (double) m_moduleZero) * ModuleConstants.kTurningEncoderDistancePerPulse;}

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurningMotorEncoderRadians()));
  }

  public void setDriveVelocity(double velocity) { m_pidController.setReference(velocity, ControlType.kVelocity); }

  /**
   * Moves the steering motor to a target and powers the drive motor
   * 
   * @param speed The speed at which to move the drive motor
   * @param targetAngle The target angle of the steering module in degrees
   */
  public void setState(double speed, double targetAngle, boolean useVelocity) {
    // Calculate the alternate target angle
    double targetAngle2 = targetAngle + 180.0;

    // Make the angle input from the -180 - 180 range to the 0 - 360 range
    if(targetAngle < 0) { targetAngle += 360.0; }

    // Get current angle and normalize it to a 0 - 360 range
    double currentAngle = (m_steerMotor.getSelectedSensorPosition() - (double) m_moduleZero) * (360.0 / (double) ModuleConstants.kTurningEncoderCPR);
    while(currentAngle > 360.0) {
      currentAngle -= 360.0;
    }
    while(currentAngle < 0.0) {
      currentAngle += 360.0;
    }
    
    // Calculate shortest angle to target
    double delta1 = 0;
    double delta2 = 0;
    if(currentAngle > targetAngle) {
      // cw delta
      delta1 = 360.0 - currentAngle + targetAngle;
      // ccw delta
      delta2 = -(currentAngle - targetAngle);  
    } else if(targetAngle > currentAngle) {
      // cw delta
      delta1 = targetAngle - currentAngle;
      // ccw delta
      delta2 = -(360.0 - targetAngle + currentAngle);
    }

    double delta = 0.0;
    if(Math.abs(delta1) < Math.abs(delta2)) {
      delta = delta1;
    } else {
      delta = delta2;
    }

    double delta3 = 0.0;
    double delta4 = 0.0;
    if(currentAngle > targetAngle2) {
      // cw delta
      delta3 = 360 - currentAngle + targetAngle2;
      // ccw delta
      delta4 = -(currentAngle - targetAngle2); 
    } else if(targetAngle2 > currentAngle) {
      // cw delta
      delta3 = targetAngle2 - currentAngle;
      // ccw delta
      delta4 = -(360 - targetAngle2 + currentAngle);
    }

    double altDelta = 0;
    if(Math.abs(delta3) < Math.abs(delta4)) {
      altDelta = delta3;
    } else {
      altDelta = delta4;
    }

    // Moves the drive motor and steering module if the module has been zeroed
    if(speed != 0.0) {
      if(Math.abs(delta) < Math.abs(altDelta)) {
        double ticksDelta = delta * ((double) ModuleConstants.kTurningEncoderCPR / 360.0);
        m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition() + ticksDelta);
        if(m_steerMotor.getClosedLoopError(ModuleConstants.kPIDLoopIdx) < 50) {
          if(useVelocity) {
            m_pidController.setReference(speed, ControlType.kVelocity);
          } else {
            m_driveMotor.set(speed);
          }
        }
      } else if(Math.abs(altDelta) < Math.abs(delta)) {
        double ticksDelta = altDelta * ((double) ModuleConstants.kTurningEncoderCPR / 360.0);
        m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition() + ticksDelta);
        if(m_steerMotor.getClosedLoopError(ModuleConstants.kPIDLoopIdx) < 50) {
          if(useVelocity) {
            m_pidController.setReference(-speed, ControlType.kVelocity);
          } else {
            m_driveMotor.set(-speed);
          }
        }
      }
    } else {
      m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition());
      m_driveMotor.set(speed);
      // m_pidController.setReference(1.0, ControlType.kVelocity);
    }
  }
}
