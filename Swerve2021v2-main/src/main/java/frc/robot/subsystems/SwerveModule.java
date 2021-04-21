// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  // Drive Motor
  // CANSparkMax m_driveMotor;
  private final TalonFX m_driveMotor;
  // Steering Motor
  private final TalonSRX m_steerMotor;

  // CANCoder
  private final CANCoder m_canCoder;

  // Module Zero
  private int m_moduleZero;

  /** 
   * Creates a new SwerveModule. 
   * 
   * @param driveMotorID ID for the drive motor.
   * @param steerMotorID ID for the steering motor.
   * @param canCoderID ID for the canCoder.
   * @param driveMotorReversed Direction of the drive motor.
   * @param steerMotorReversed Direction of the steering motor.
   * @param driveEncoderReversed Phase of the drive encoder.
   * @param steerEncoderReversed Phase of the steering encoder.
   * @param moduleZero The aboslute encoder count when the module is facing forward.
   * */
  public SwerveModule(int driveMotorID, int steerMotorID, int canCoderID, boolean driveMotorReversed, boolean steerMotorReversed, boolean driveEncoderReversed, boolean steerEncoderReversed, int moduleZero) {
    // Motor Init
    // m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_driveMotor = new TalonFX(driveMotorID);
    m_steerMotor = new TalonSRX(steerMotorID);

    // CANCoder Init
    m_canCoder = new CANCoder(canCoderID);
    m_canCoder.configSensorDirection(steerEncoderReversed);

    // Motor Reset
    m_driveMotor.configFactoryDefault();
    m_steerMotor.configFactoryDefault();

    // Motor Inversions
    m_driveMotor.setInverted(driveMotorReversed);
    m_steerMotor.setInverted(steerMotorReversed);
    
    // Motor Neutral Mode
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    // Config drive encoder
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ModuleConstants.kPIDLoopIdx, ModuleConstants.kTimeoutMs);
    // Config steer encoders
    m_steerMotor.configRemoteFeedbackFilter(m_canCoder, ModuleConstants.kRemoteID, ModuleConstants.kTimeoutMs);
    m_steerMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, ModuleConstants.kPIDLoopIdx, ModuleConstants.kTimeoutMs);

    // Steering Encoder Sensor Phase
    m_driveMotor.setSensorPhase(driveEncoderReversed);
    // m_steerMotor.setSensorPhase(steerEncoderReversed);

    // Drive motor open loop ramp rate
    m_driveMotor.configOpenloopRamp(ModuleConstants.kOpenRampRate, ModuleConstants.kTimeoutMs);

    // Set relevant frame periods to be at least as fast as periodic rate
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ModuleConstants.kTimeoutMs);
    m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ModuleConstants.kTimeoutMs);
		m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ModuleConstants.kTimeoutMs);

    // Set the peak and nominal outputs
    m_driveMotor.configNominalOutputForward(0, ModuleConstants.kTimeoutMs);
		m_driveMotor.configNominalOutputReverse(0, ModuleConstants.kTimeoutMs);
		m_driveMotor.configPeakOutputForward(1, ModuleConstants.kTimeoutMs);
		m_driveMotor.configPeakOutputReverse(-1, ModuleConstants.kTimeoutMs);
		m_steerMotor.configNominalOutputForward(0, ModuleConstants.kTimeoutMs);
		m_steerMotor.configNominalOutputReverse(0, ModuleConstants.kTimeoutMs);
		m_steerMotor.configPeakOutputForward(1, ModuleConstants.kTimeoutMs);
    m_steerMotor.configPeakOutputReverse(-1, ModuleConstants.kTimeoutMs);
    
    // Set Drive Velocity gains in slot0
    m_driveMotor.selectProfileSlot(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDrivePIDLoopIdx);
    m_driveMotor.config_kF(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kF, ModuleConstants.kTimeoutMs);
		m_driveMotor.config_kP(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kP, ModuleConstants.kTimeoutMs);
		m_driveMotor.config_kI(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kI, ModuleConstants.kTimeoutMs);
    m_driveMotor.config_kD(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kD, ModuleConstants.kTimeoutMs);
    m_driveMotor.config_IntegralZone(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kIzone, ModuleConstants.kTimeoutMs);

		// Set Motion Magic gains in slot0
		m_steerMotor.selectProfileSlot(ModuleConstants.kSlotIdx, ModuleConstants.kPIDLoopIdx);
		m_steerMotor.config_kF(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kF, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kP(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kP, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kI(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kI, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_kD(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kD, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_IntegralZone(ModuleConstants.kSlotIdx, ModuleConstants.kGains.kIzone, ModuleConstants.kTimeoutMs);

    // Set Motion Magic gains in slot1
    // TODO check if PIDLoopID can be set to 0, maybe changing slot and loop to aux makes motion magic work?
		m_steerMotor.config_kF(ModuleConstants.kAuxSlotIdx, ModuleConstants.kAuxGains.kF, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kP(ModuleConstants.kAuxSlotIdx, ModuleConstants.kAuxGains.kP, ModuleConstants.kTimeoutMs);
		m_steerMotor.config_kI(ModuleConstants.kAuxSlotIdx, ModuleConstants.kAuxGains.kI, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_kD(ModuleConstants.kAuxSlotIdx, ModuleConstants.kAuxGains.kD, ModuleConstants.kTimeoutMs);
    m_steerMotor.config_IntegralZone(ModuleConstants.kAuxSlotIdx, ModuleConstants.kAuxGains.kIzone, ModuleConstants.kTimeoutMs);

		// Set acceleration and cruise velocity
		m_steerMotor.configMotionCruiseVelocity(ModuleConstants.MODULE_CRUISE_VELOCITY, ModuleConstants.kTimeoutMs);
    m_steerMotor.configMotionAcceleration(ModuleConstants.MODULE_ACCELERATION, ModuleConstants.kTimeoutMs);

    m_moduleZero = moduleZero;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Gives the position of the drive motor in meters
   * 
   * @return The position of the drive motor in meters
   */
  public double getDrivePosition() { return m_driveMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx) * ModuleConstants.kDriveEncoderDistancePerTick; }

  /**
   * Gives the speed of the drive motor in meters per second
   * 
   * @return The speed of the drive motor in meters per second
   */
  public double getDriveVelocity() { return m_driveMotor.getSelectedSensorVelocity(ModuleConstants.kPIDLoopIdx) * ModuleConstants.kDriveEncoderMetersPerSecondPerEncoderTickPer100Ms; }

  /**
   * Gives the angle of the steering motor in radians
   * 
   * @return The angle of the steering motor in radians
   */
  public double getTurningMotorEncoderRadians() { return (m_steerMotor.getSelectedSensorPosition() - (double) m_moduleZero) * ModuleConstants.kTurningEncoderDistancePerPulse;}
  // public double getTurningMotorEncoderRadians() { return m_steerMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx) - (double) m_moduleZero; }

  /**
   * Gives the current state of the swerve module
   * 
   * @return The state of the swerve module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(getTurningMotorEncoderRadians()));
  }

  /**
   * Sets the drive velocity of the drive motor
   * 
   * @param velocity The velocity to be set in meters per second
   */
  public void setDriveVelocity(double velocity) { m_driveMotor.set(ControlMode.Velocity, velocity / ModuleConstants.kDriveEncoderMetersPerSecondPerEncoderTickPer100Ms); }

  /**
   * Moves the steering motor to a target and powers the drive motor
   * 
   * @param speed The speed at which to move the drive motor
   * @param targetAngle The target angle of the steering module in degrees
   * @param useVelocity Boolean if the drive motor should set with velocity: "true" or percent power: "false"
   */
  public void setState(double speed, double targetAngle, boolean useVelocity) {
    // Calculate the alternate target angle
    double targetAngle2 = targetAngle + 180.0;

    // Make the angle input from the -180 - 180 range to the 0 - 360 range
    if(targetAngle < 0) { targetAngle += 360.0; }

    // Get current angle and normalize it to a 0 - 360 range
    double currentAngle = (m_steerMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx) - (double) m_moduleZero) * (360.0 / (double) ModuleConstants.kTurningEncoderCPR);
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
        m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx) + ticksDelta);
        // if(m_steerMotor.getClosedLoopError(ModuleConstants.kPIDLoopIdx) < 50) {
        if(useVelocity) {
          this.setDriveVelocity(speed);
        } else {
          m_driveMotor.set(ControlMode.PercentOutput, speed);
        }
        // }
      } else if(Math.abs(altDelta) < Math.abs(delta)) {
        double ticksDelta = altDelta * ((double) ModuleConstants.kTurningEncoderCPR / 360.0);
        m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx) + ticksDelta);
        // if(m_steerMotor.getClosedLoopError(ModuleConstants.kPIDLoopIdx) < 50) {
        if(useVelocity) {
          this.setDriveVelocity(-speed);
        } else {
          m_driveMotor.set(ControlMode.PercentOutput, -speed);
        }
        // }
      }
    } else {
      m_steerMotor.set(ControlMode.MotionMagic, m_steerMotor.getSelectedSensorPosition(ModuleConstants.kPIDLoopIdx));
      m_driveMotor.set(ControlMode.PercentOutput, speed);
      // m_pidController.setReference(1.0, ControlType.kVelocity);
    }
  }
}
