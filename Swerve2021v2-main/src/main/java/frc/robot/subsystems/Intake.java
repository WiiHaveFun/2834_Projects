// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Motor
  private final CANSparkMax m_motor;

  /** Creates a new Intake. */
  public Intake(int motorID, boolean motorReversed) {
    m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
    m_motor.setInverted(motorReversed);
    m_motor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the power of the motor
   * 
   * @param power the power to set to
   */
  public void setPower(double power) {
    m_motor.set(power);
  }
}
