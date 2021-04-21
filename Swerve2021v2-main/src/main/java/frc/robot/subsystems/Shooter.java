// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // Motors
  TalonFX m_flyLeft;
  TalonFX m_flyRight;
  CANSparkMax m_turret;

  // Servos
  Servo m_hoodLeft;
  Servo m_hoodRight;

  // Encoders
  CANEncoder m_turretEncoder;
  Encoder m_hoodEncoder;

  // Turret PID Controller
  CANPIDController m_turretController;

  // Hood PID Controller
  PIDController m_hoodController;

  int index = 0;
  List<Double> angleList = new ArrayList<Double>();
  List<Double> rpmList = new ArrayList<Double>();
  double totalDataPoints = 10.0;

  /** Creates a new Shooter. */
  public Shooter(int flyLeftID, int flyRightID, int hoodLeftID, int hoodRightID, int turretID, boolean flyLeftReversed, boolean flyRightReversed, boolean turretReversed, boolean flyEncoderReversed) {
    m_flyLeft = new TalonFX(flyLeftID);
    m_flyRight = new TalonFX(flyRightID);
    m_turret = new CANSparkMax(turretID, MotorType.kBrushless);

    m_hoodLeft = new Servo(hoodLeftID);
    m_hoodRight = new Servo(hoodRightID);
    
    m_hoodEncoder = new Encoder(0, 1, false, EncodingType.k1X);
    m_hoodController = new PIDController(ShooterConstants.kPHood, ShooterConstants.kIHood, ShooterConstants.kDHood);

    // Factory default all motors
    m_flyLeft.configFactoryDefault();
    m_flyRight.configFactoryDefault();
    
    // Motor inversions
    m_flyLeft.setInverted(flyLeftReversed);
    m_flyRight.setInverted(flyRightReversed);
    m_turret.setInverted(turretReversed);

    // Flywheel neutral mode
    m_flyLeft.setNeutralMode(NeutralMode.Coast);
    m_flyRight.setNeutralMode(NeutralMode.Coast);

    // Flywheel follower
    m_flyRight.follow(m_flyLeft);

    // Config flywheel encoder
    m_flyLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.kFlyPIDLoopIdx, ShooterConstants.kTimeoutMs);
    m_flyLeft.setSensorPhase(flyEncoderReversed);

    // Encoder Sensor Phase
    m_flyLeft.setSensorPhase(flyEncoderReversed);

    // Set relevant frame periods to be at least as fast as periodic rate
    m_flyLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ShooterConstants.kTimeoutMs);

    // Closed loop ramp rate
    m_flyLeft.configClosedloopRamp(ShooterConstants.kClosedLoopRampRate, ShooterConstants.kTimeoutMs);

    // Set the peak and nominal outputs
    m_flyLeft.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		m_flyLeft.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		m_flyLeft.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    m_flyLeft.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);
    
    // Set Drive Velocity gains in slot0
    m_flyLeft.selectProfileSlot(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyPIDLoopIdx);
    m_flyLeft.config_kF(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyGains.kF, ShooterConstants.kTimeoutMs);
		m_flyLeft.config_kP(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyGains.kP, ShooterConstants.kTimeoutMs);
		m_flyLeft.config_kI(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyGains.kI, ShooterConstants.kTimeoutMs);
    m_flyLeft.config_kD(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyGains.kD, ShooterConstants.kTimeoutMs);
    m_flyLeft.config_IntegralZone(ShooterConstants.kFlySlotIdx, ShooterConstants.kFlyGains.kIzone, ShooterConstants.kTimeoutMs);
 
    // Turret neutral mode
    m_turret.setIdleMode(IdleMode.kBrake);

    // Turret PID config
    m_turretController = m_turret.getPIDController();
    m_turretEncoder = m_turret.getEncoder();
    m_turretEncoder.setPositionConversionFactor(ShooterConstants.turretDegreesPerMotorRot);
    m_turretEncoder.setVelocityConversionFactor(ShooterConstants.turretDegPerSecPerMotorRMP);
  
    // set turret PID coefficients
    m_turretController.setP(ShooterConstants.kGains.kP);
    m_turretController.setI(ShooterConstants.kGains.kI);
    m_turretController.setD(ShooterConstants.kGains.kD);
    m_turretController.setIZone(ShooterConstants.kGains.kIzone);
    m_turretController.setFF(ShooterConstants.kGains.kF);
    m_turretController.setOutputRange(-ShooterConstants.kGains.kPeakOutput, ShooterConstants.kGains.kPeakOutput);

    m_turretController.setSmartMotionMaxVelocity(ShooterConstants.MODULE_CRUISE_VELOCITY, ShooterConstants.kTurretSlotIdx);
    m_turretController.setSmartMotionMinOutputVelocity(-ShooterConstants.MODULE_CRUISE_VELOCITY, ShooterConstants.kTurretSlotIdx);
    m_turretController.setSmartMotionMaxAccel(ShooterConstants.MODULE_ACCELERATION, ShooterConstants.kTurretSlotIdx);
    m_turretController.setSmartMotionAllowedClosedLoopError(ShooterConstants.kAllowedError, ShooterConstants.kTurretSlotIdx);

    SmartDashboard.putNumber("Man_RPM", 0);
    SmartDashboard.putNumber("Man_Hood", 0);
    SmartDashboard.putNumber("Man_Turret", 0);
    SmartDashboard.putBoolean("Manual_Turret", false);
    SmartDashboard.putBoolean("Manual_Hood_Set", false);
    SmartDashboard.putBoolean("Manual_Turret_Set", false);

    m_hoodEncoder.reset();
    m_turretEncoder.setPosition(0);

    for(int i=0; i<totalDataPoints; i++) {
      angleList.add(0.0);
      rpmList.add(0.0);
    }
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RPM", m_flyLeft.getSelectedSensorVelocity() / ShooterConstants.RPMtoTicksPer100ms);
    SmartDashboard.putNumber("Hood_Position", m_hoodEncoder.get());
    SmartDashboard.putNumber("Turret_Position", m_turretEncoder.getPosition());
    SmartDashboard.putNumber("Turret_Velocity", m_turretEncoder.getVelocity());

    angleList.add(index, SmartDashboard.getNumber("targetAngle", 0.0));
    rpmList.add(index, SmartDashboard.getNumber("targetRPM", 0.0));
    if(index == totalDataPoints - 1.0) {
      index = 0;
    } else {
      index++;
    }

    
    double sum = 0;
    double rpmSum = 0;
    for(int i=0; i<totalDataPoints; i++) {
      sum+=angleList.get(i);
      rpmSum+=rpmList.get(i);
    }
    SmartDashboard.putNumber("targetAngleAve", sum/totalDataPoints);
    SmartDashboard.putNumber("targetRPMAve", rpmSum/totalDataPoints);
  }

  public void setFlyPower(double power) {
    m_flyLeft.set(ControlMode.PercentOutput, power);
  }

  public void setFlySpeed(double RPM) {
    double ticksPer100ms = RPM * ShooterConstants.RPMtoTicksPer100ms;
    if(RPM != 0.0) {
      m_flyLeft.set(ControlMode.Velocity, ticksPer100ms);
    } else {
      m_flyLeft.set(ControlMode.PercentOutput, 0.0);
    }
    
  }

  public void moveHood(double demand) {
    m_hoodLeft.set(0.5 + demand * 0.4);
    m_hoodRight.set(0.5 - demand * 0.4);
  }

  public void setHoodPosition(int position) {
    m_hoodController.setSetpoint(position);
    double demand = MathUtil.clamp(m_hoodController.calculate(m_hoodEncoder.get()), -1.0, 1.0);
    moveHood(demand);
  }

  public void moveTurret(double power) {
    m_turret.set(power);
  }

  public void setTurretAngle(double angle) {
    m_turretController.setReference(angle, ControlType.kSmartMotion);
  }

  public double getTurretAngle() {
    return m_turretEncoder.getPosition();
  }

  public void resetHoodEncoder() {
    m_hoodEncoder.reset();
  }
}
